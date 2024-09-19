'''vscode.py

Generation of c_cpp_properties.json and launch.json'''

import sys
import shutil
from pathlib import PurePath, Path
from json import dumps
from textwrap import dedent

from west.commands import WestCommand
from west.configuration import config

from cognitag_eip import argparse_helpers
from cognitag_eip.zephyr.zbuild import ZephyrBuildConfig

soc_mapping = {
    "nRF5340_CPUAPP_QKAA": "nrf5340_xxAA_app",
    "nRF5340_CPUNET_QKAA": "nrf5340_xxAA_net",
    "nRF52832_QFAA": "nrf52832_xxAA",
    "nRF52833_QIAA": "nrf52833_xxAA",
    "nRF52840_QIAA": "nrf52840_xxAA",
    "nRF9160_SICA": "nrf9160_xxAA"
}

mdk_dir = Path("modules") / "hal" / "nordic" / "nrfx" / "mdk"
svd_files = {
    "nrf52832_xxAA": mdk_dir / "nrf52832.svd",
    "nrf52833_xxAA": mdk_dir / "nrf52833.svd",
    "nrf52840_xxAA": mdk_dir / "nrf52840.svd",
    "nrf5340_xxAA_app": mdk_dir / "nrf5340_application.svd",
    "nrf5340_xxAA_net": mdk_dir / "nrf5340_network.svd",
    "nrf9160_xxAA": mdk_dir / "nrf9160.svd"
}

extensions_template = {
    "recommendations": [
        'ms-vscode.cpptools',
        'ms-vscode.cpptools-extension-pack',
        'ms-vscode.cmake-tools',
        'ms-python.python',
        'nordic-semiconductor.nrf-devicetree',
        'nordic-semiconductor.nrf-kconfig',
        'trond-snekvik.gnu-mapfiles',
        'trond-snekvik.simple-rst',
        'marus25.cortex-debug',
        'zachflower.uncrustify',
        'lextudio.restructuredtext',
	]
}

settings_template = {
    "uncrustify.configPath.windows": "eip-sdk/scripts/uncrustify.cfg",
    "uncrustify.configPath.osx": "eip-sdk/scripts/uncrustify.cfg",
    "uncrustify.configPath.linux": "eip-sdk/scripts/uncrustify.cfg",
    "editor.formatOnSave": True,
    "editor.defaultFormatter": "zachflower.uncrustify",
    "editor.trimAutoWhitespace": True,
    "editor.insertSpaces": False,
    "editor.tabSize": 8,
    "[git-commit]": {"editor.rulers": [72]},
    "[c]": {"editor.rulers": [120]},
    "files.trimFinalNewlines": True,
    "files.insertFinalNewline": True,
    "files.associations": {
        "COMMIT_EDITMSG": "git-commit",
        "git-rebase-todo": "git-rebase"
    },
    "C_Cpp.files.exclude": {
        "**/generated/syscall_links/**": True
    }
}

c_cpp_properties_template = {
    "configurations": [
        {
            "name": "Zephyr",
            "cStandard": "c99",
            "compilerPath": "",
            "intelliSenseMode": "gcc-x64",
            "compileCommands": "",
            "includePath": [
                "zephyr/include",
                "eip-sdk/include"
            ]
        }
    ],
    "version": 4
}

launch_template = {
    "version": "0.2.0",
    "configurations": [
        {
            "name": "jlink attach",
            "cwd": "${workspaceRoot}",
            "executable": "",
            "request": "attach",
            "preAttachCommands": [],
            "type": "cortex-debug",
            "device": "",
            "rtos": "Zephyr",
            "servertype": "jlink"
        },
        {
            "name": "jlink launch",
            "cwd": "${workspaceRoot}",
            "executable": "",
            "request": "launch",
            "breakAfterReset": True,
            "type": "cortex-debug",
            "device": "",
            "rtos": "Zephyr",
            "servertype": "jlink"
        },
        {
            "name": "openocd attach",
            "cwd": "${workspaceRoot}",
            "executable": "",
            "request": "attach",
            "preAttachCommands": [],
            "type": "cortex-debug",
            "preLaunchTask": "start openocd",
            "device": "",
            "servertype": "external",
            "gdbTarget": "localhost:3333"
        }
    ]
}

c_snippets = {
    "New_H_File": {
        "prefix": "new_h_file",
        "body": [
            "/**",
            " * @file",
            " * @brief ${1:Brief description.}",
            " * @author $SNIPPET_AUTHOR$",
            " * @copyright $SNIPPET_COPYRIGHT$, $CURRENT_YEAR",
            " * @spdxlicense $SNIPPET_LICENSE$",
            " *",
            " * @details",
            " */",
            "",
            "#ifndef $SNIPPET_PROJECT$_INCLUDE_${TM_DIRECTORY/.*[\\\\|\\/]+(.*)/${1:/upcase}/}_${TM_FILENAME_BASE/(.*)/${1:/upcase}/}_H_",
            "#define $SNIPPET_PROJECT$_INCLUDE_${TM_DIRECTORY/.*[\\\\|\\/]+(.*)/${1:/upcase}/}_${TM_FILENAME_BASE/(.*)/${1:/upcase}/}_H_",
            "",
            "/* Includes ------------------------------------------------------------------*/",
            "",
            "",
            "#ifdef __cplusplus",
            "extern \"C\" {",
            "#endif",
            "",
            "/* Module Defines ------------------------------------------------------------*/",
            "",
            "/* Type Definitions ----------------------------------------------------------*/",
            "",
            "/* Function Declarations -----------------------------------------------------*/",
            "",
            "/*----------------------------------------------------------------------------*/",
            "",
            "#ifdef __cplusplus",
            "}",
            "#endif",
            "",
            "#endif /* $SNIPPET_PROJECT$_INCLUDE_${TM_DIRECTORY/.*[\\\\|\\/]+(.*)/${1:/upcase}/}_${TM_FILENAME_BASE/(.*)/${1:/upcase}/}_H_ */"
        ],
        "description": "Create template for new H file."
    },
    "New_C_File": {
        "prefix": "new_c_file",
        "body": [
            "/**",
            " * @file",
            " * @author $SNIPPET_AUTHOR$",
            " * @copyright $SNIPPET_COPYRIGHT$, $CURRENT_YEAR",
            " * @spdxlicense $SNIPPET_LICENSE$",
            " */",
            "",
            "/* Includes ------------------------------------------------------------------*/",
            "",
            "/* Private Defines -----------------------------------------------------------*/",
            "",
            "/* Type Definitions ----------------------------------------------------------*/",
            "",
            "/* Function Declarations -----------------------------------------------------*/",
            "",
            "/* Private Variables ---------------------------------------------------------*/",
            "",
            "/*----------------------------------------------------------------------------*/"
        ],
        "description": "Create template for new C file."
    },
    "New_RPC_File": {
        "prefix": "new_rpc_file",
        "body": [
            "/**",
            " * @file",
            " * @author $SNIPPET_AUTHOR$",
            " * @copyright $SNIPPET_COPYRIGHT$, $CURRENT_YEAR",
            " * @spdxlicense $SNIPPET_LICENSE$",
            " */",
            "",
            "/* Includes ------------------------------------------------------------------*/",
            "",
            "#include <eip/rpc/rpc_struct.h>",
            "#include <eip/rpc/rpc.h>",
            "",
            "/*----------------------------------------------------------------------------*/",
            "",
            "int rpc_${1:name}(struct ${1:name}_in *in, struct ${1:name}_out *out)",
            "{",
            "\tARG_UNUSED(in);",
            "\tARG_UNUSED(out);",
            "\treturn -EINVAL;",
            "}",
            "",
            "/*----------------------------------------------------------------------------*/"
        ],
        "description": "Create template for new RPC command."
    }
}

class vscode(WestCommand):

    def __init__(self):
        super().__init__(
            'vscode',  # gets stored as self.name
            'generate Visual Studio code configuration files',  # self.help
            # self.description:
            dedent('''
            Generate Visual Studio code configuration files.

            When provided with a build directory, generates intellisense and
            debugging configuration files.
            Without a build directory, generates extensions recommendations,
            C code snippets and editor settings.'''))

    def do_add_parser(self, parser_adder):
        parser = parser_adder.add_parser(self.name,
                                         help=self.help,
                                         description=self.description)

        parser.add_argument('-d', '--build-dir', type=argparse_helpers.ValidPath, help='Zephyr build directory')
        parser.add_argument('-o', '--output-dir', default=Path('.'), type=argparse_helpers.ValidPath, help='Folder containing .vscode directory')

        return parser

    def do_run(self, args, unknown_args):
        # C Snippets parameterisation
        def snippet_template_set(field, value):
            c_snippets['New_C_File']['body'] = [l.replace(field, value) for l in c_snippets['New_C_File']['body']]
            c_snippets['New_H_File']['body'] = [l.replace(field, value) for l in c_snippets['New_H_File']['body']]
            c_snippets['New_RPC_File']['body'] = [l.replace(field, value) for l in c_snippets['New_RPC_File']['body']]

        snippet_project = config.get('eip', 'c-snippets-project', fallback="EIP")
        snippet_author = config.get('eip', 'c-snippets-author', fallback="${2:Name} <${3:Email}>")
        snippet_copyright = config.get('eip', 'c-snippets-copyright', fallback="${4:Copyright}")
        snippet_license = config.get('eip', 'c-snippets-license', fallback="Apache-2.0")

        snippet_template_set("$SNIPPET_PROJECT$", snippet_project)
        snippet_template_set("$SNIPPET_AUTHOR$", snippet_author)
        snippet_template_set("$SNIPPET_COPYRIGHT$", snippet_copyright)
        snippet_template_set("$SNIPPET_LICENSE$", snippet_license)

        vscode_dir = (args.output_dir / '.vscode').resolve()
        c_cpp_properties_file = vscode_dir / 'c_cpp_properties.json'
        settings_file = vscode_dir / 'settings.json'
        extensions_file = vscode_dir / 'extensions.json'
        launch_file = vscode_dir / 'launch.json'
        snippets_file = vscode_dir / 'c.code-snippets'

        vscode_dir.mkdir(exist_ok=True)

        if args.build_dir is not None:
            # Application specific configuration
            try:
                build = ZephyrBuildConfig(args.build_dir)
            except FileNotFoundError:
                sys.exit(f"'{args.build_dir}' does not appear to be a valid Zephyr build folder")

            # autoconf.h variables
            configs = build.find_configs(["SOC"], ["EIP_SECURE_BOOT_APPLICATION"])
            soc = configs["SOC"]

            # c_cpp_properties.json variables
            compiler_path = PurePath(build.cache['CMAKE_C_COMPILER'])
            compile_commands_path = PurePath('${workspaceFolder}') / args.build_dir / 'compile_commands.json'
            generated_includes_path = PurePath('${workspaceFolder}') / args.build_dir / 'zephyr' / 'include' / 'generated' / 'zephyr'

            # launch.json variables
            try:
                dev_type = soc_mapping[soc]
            except KeyError:
                dev_type = ""
            elf_file = PurePath(args.build_dir) / 'zephyr' / 'zephyr.elf'
            svd_file = svd_files.get(dev_type, None)

            c_cpp_properties_template['configurations'][0]['compilerPath'] = compiler_path.as_posix()
            c_cpp_properties_template['configurations'][0]['compileCommands'] = compile_commands_path.as_posix()
            c_cpp_properties_template['configurations'][0]['includePath'].append(generated_includes_path.as_posix())

            def launch_template_set(field, value, exclude=[]):
                for i in [0,1,2]:
                    if i not in exclude:
                        launch_template['configurations'][i][field] = value

            launch_template_set('device', dev_type)
            launch_template_set('executable', elf_file.as_posix())
            if svd_file:
                launch_template_set('svdFile', svd_file.as_posix())
            if "EIP_SECURE_BOOT_APPLICATION" in configs:
                secure_app = configs["EIP_SECURE_BOOT_APPLICATION"]
                secure_elf = PurePath(args.build_dir) / secure_app / "zephyr" / "zephyr.elf"
                command = f"add-symbol-file {secure_elf.as_posix()}"
                launch_template_set('preAttachCommands', [command], exclude=[1])
            if "CMAKE_GDB" in build.cache and Path(build.cache['CMAKE_GDB']).exists():
                gdb_path_split = build.cache['CMAKE_GDB'].rsplit('/', 1)
                gdb_executable = gdb_path_split[-1]
                if gdb_executable[-2:] == 'py':
                    toolchain_prefix = gdb_executable.rsplit('-', 2)[0]
                else:
                    toolchain_prefix = gdb_executable.rsplit('-', 1)[0]
                launch_template_set('toolchainPrefix', toolchain_prefix)
                launch_template_set('armToolchainPath', gdb_path_split[0])
                launch_template_set('gdbPath', build.cache['CMAKE_GDB'])

            c_cpp_properties_file.write_text(dumps(c_cpp_properties_template, indent=4))
            launch_file.write_text(dumps(launch_template, indent=4))

            print(f"VSCode config for '{args.build_dir}' written to '{vscode_dir}'")
        else:
            # Configure the default Python interpreter (Will detect virtual environments)
            python_path = shutil.which('python3') or shutil.which('python')
            if python_path is not None:
                settings_template['python.defaultInterpreterPath'] = python_path
            # General vscode configuration
            extensions_file.write_text(dumps(extensions_template, indent=4))
            settings_file.write_text(dumps(settings_template, indent=4))
            snippets_file.write_text(dumps(c_snippets, indent=4))

            print(f"VScode extension recommendations, snippets and settings written to '{vscode_dir}'")
