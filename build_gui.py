import sys
import os
import re
import subprocess
import platform
import json
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QComboBox, QPushButton, QTabWidget, QFormLayout, QCheckBox,
    QLineEdit, QPlainTextEdit, QLabel, QMessageBox,
    QScrollArea, QInputDialog, QGridLayout, QSizePolicy, QLayout,
    QGroupBox, QFrame
)
from PyQt6.QtCore import QProcess, QProcessEnvironment, Qt, QPoint, QRect, QSize

class FlowLayout(QLayout):
    def __init__(self, parent=None, margin=-1, hspacing=-1, vspacing=-1):
        super().__init__(parent)
        self._hspacing = hspacing
        self._vspacing = vspacing
        self._items = []
        self.setContentsMargins(margin, margin, margin, margin)

    def __del__(self):
        del self._items

    def addItem(self, item):
        self._items.append(item)

    def horizontalSpacing(self):
        if self._hspacing >= 0:
            return self._hspacing
        else:
            return self.smartSpacing(Qt.Orientation.Horizontal)

    def verticalSpacing(self):
        if self._vspacing >= 0:
            return self._vspacing
        else:
            return self.smartSpacing(Qt.Orientation.Vertical)

    def count(self):
        return len(self._items)

    def itemAt(self, index):
        if 0 <= index < len(self._items):
            return self._items[index]
        return None

    def takeAt(self, index):
        if 0 <= index < len(self._items):
            return self._items.pop(index)
        return None

    def expandingDirections(self):
        return Qt.Orientation(0)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        height = self.doLayout(QRect(0, 0, width, 0), True)
        return height

    def setGeometry(self, rect):
        super().setGeometry(rect)
        self.doLayout(rect, False)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        size = QSize()
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        size += QSize(2 * self.contentsMargins().top(), 2 * self.contentsMargins().top())
        return size

    def doLayout(self, rect, test_only):
        left, top, right, bottom = self.getContentsMargins()
        effective_rect = rect.adjusted(+left, +top, -right, -bottom)
        x = effective_rect.x()
        y = effective_rect.y()
        line_height = 0

        for item in self._items:
            wid = item.widget()
            space_x = self.horizontalSpacing()
            if space_x == -1:
                space_x = wid.style().layoutSpacing(QSizePolicy.ControlType.PushButton, QSizePolicy.ControlType.PushButton, Qt.Orientation.Horizontal)
            space_y = self.verticalSpacing()
            if space_y == -1:
                space_y = wid.style().layoutSpacing(QSizePolicy.ControlType.PushButton, QSizePolicy.ControlType.PushButton, Qt.Orientation.Vertical)
            
            next_x = x + item.sizeHint().width() + space_x
            if next_x - space_x > effective_rect.right() and line_height > 0:
                x = effective_rect.x()
                y = y + line_height + space_y
                next_x = x + item.sizeHint().width() + space_x
                line_height = 0

            if not test_only:
                item.setGeometry(QRect(QPoint(x, y), item.sizeHint()))

            x = next_x
            line_height = max(line_height, item.sizeHint().height())

        return y + line_height - rect.y() + bottom

    def smartSpacing(self, orientation):
        parent = self.parent()
        if not parent:
            return -1
        elif parent.isWidgetType():
            return parent.style().layoutSpacing(QSizePolicy.ControlType.PushButton, QSizePolicy.ControlType.PushButton, orientation)
        else:
            return parent.spacing()

class LiteXBuildGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiteX Build GUI - LimeSDR_GW")
        self.resize(1000, 800)

        self.venv_python = self.find_venv_python()
        self.target_args = {}
        self.toolchain_env_scripts = []
        self.current_target_path = None

        self.init_ui()
        self.scan_targets()

    def find_venv_python(self):
        """Locates the Python executable within the .venv directory."""
        venv_dir = os.path.join(os.getcwd(), ".venv")
        if platform.system() == "Windows":
            python_exe = os.path.join(venv_dir, "Scripts", "python.exe")
        else:
            python_exe = os.path.join(venv_dir, "bin", "python")
        
        if os.path.exists(python_exe):
            return python_exe
        return None

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Workspace Configuration Group
        workspace_group = QGroupBox("Workspace Configuration")
        workspace_layout = QVBoxLayout(workspace_group)
        
        # Top Bar: Setup and Settings
        top_bar = QHBoxLayout()
        self.setup_button = QPushButton("Run setup_litex.sh")
        self.setup_button.clicked.connect(self.run_setup_litex)
        if self.venv_python:
            self.setup_button.setEnabled(False)
            self.setup_button.setToolTip("Virtual environment already exists.")
        
        self.settings_button = QPushButton("Toolchain Settings")
        self.settings_button.clicked.connect(self.show_settings)
        
        top_bar.addWidget(self.setup_button)
        top_bar.addStretch()
        top_bar.addWidget(self.settings_button)
        workspace_layout.addLayout(top_bar)

        # Target Selection
        target_layout = QHBoxLayout()
        target_layout.addWidget(QLabel("Target:"))
        self.target_combo = QComboBox()
        self.target_combo.activated.connect(self.on_target_selected)
        self.target_combo.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Fixed)
        self.target_combo.setMinimumWidth(250)
        target_layout.addWidget(self.target_combo)
        target_layout.addStretch()
        workspace_layout.addLayout(target_layout)
        
        layout.addWidget(workspace_group)

        # Dynamic Options Tabs
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)

        # Build and Stop Buttons (Right-aligned)
        buttons_outer_layout = QHBoxLayout()
        buttons_outer_layout.addStretch()
        
        buttons_layout = QHBoxLayout()
        buttons_layout.setContentsMargins(0, 10, 0, 10)
        buttons_layout.setSpacing(10)
        
        self.build_button = QPushButton("Build")
        self.build_button.clicked.connect(self.start_build)
        self.build_button.setFixedSize(120, 40)
        self.build_button.setStyleSheet("font-weight: bold; background-color: #4CAF50; color: white; border-radius: 5px;")
        
        self.stop_button = QPushButton("Stop Build")
        self.stop_button.clicked.connect(self.stop_build)
        self.stop_button.setFixedSize(120, 40)
        self.stop_button.setStyleSheet("font-weight: bold; background-color: #f44336; color: white; border-radius: 5px;")
        self.stop_button.setEnabled(False)
        
        buttons_layout.addWidget(self.build_button)
        buttons_layout.addWidget(self.stop_button)
        buttons_outer_layout.addLayout(buttons_layout)
        layout.addLayout(buttons_outer_layout)

        # Embedded Terminal
        layout.addWidget(QLabel("Output Log:"))
        self.terminal = QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: 'Consolas', 'Monaco', monospace; font-size: 12px; border: 1px solid #333;")
        layout.addWidget(self.terminal, 1)

        # Process management
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.on_stdout)
        self.process.readyReadStandardError.connect(self.on_stderr)
        self.process.finished.connect(self.on_process_finished)

    def scan_targets(self):
        targets_dir = os.path.join(os.getcwd(), "boards", "targets")
        if not os.path.exists(targets_dir):
            self.log(f"Error: Targets directory not found at {targets_dir}")
            return

        self.target_combo.clear()
        for f in sorted(os.listdir(targets_dir)):
            if f.endswith(".py") and f != "__init__.py":
                self.target_combo.addItem(f, os.path.join(targets_dir, f))

    def on_target_selected(self):
        self.current_target_path = self.target_combo.currentData()
        if not self.current_target_path:
            return
        
        if not self.venv_python:
            # Block parsing if venv is missing
            self.tabs.clear()
            self.tabs.addTab(QLabel("Virtual environment not found. Please run setup_litex.sh first."), "ERROR")
            QMessageBox.warning(self, "No VENV", "Virtual environment not found. Please run setup_litex.sh first.")
            return

        self.log(f"Parsing target: {os.path.basename(self.current_target_path)}")
        self.parse_target_options(self.current_target_path)

    def parse_target_options(self, target_path):
        # Run our extraction tool: python tools/target_info.py <target_path>
        try:
            # Using the same python as for targets if available
            venv_dir = None
            if self.venv_python:
                venv_dir = os.path.dirname(os.path.dirname(self.venv_python))

            info_tool = os.path.join(os.getcwd(), "tools", "target_info.py")
            if not os.path.exists(info_tool):
                self.log(f"Error: Extraction tool not found at {info_tool}")
                QMessageBox.critical(self, "Missing Tool", f"Extraction tool not found at {info_tool}")
                return

            # Prepare command wrapping for activation
            if venv_dir:
                if platform.system() == "Windows":
                    activate_cmd = f"call {os.path.join(venv_dir, 'Scripts', 'activate.bat')}"
                    full_cmd = f"{activate_cmd} && python {info_tool} {target_path}"
                    shell_exe = "cmd"
                    shell_args = ["/c", full_cmd]
                else:
                    activate_cmd = f"source {os.path.join(venv_dir, 'bin', 'activate')}"
                    full_cmd = f"{activate_cmd} && python3 {info_tool} {target_path}"
                    shell_exe = "bash"
                    shell_args = ["-c", full_cmd]
            else:
                python_bin = "python3"
                shell_exe = python_bin
                shell_args = [info_tool, target_path]

            env = os.environ.copy()
            env["PYTHONPATH"] = os.getcwd()
            if venv_dir:
                env["VIRTUAL_ENV"] = venv_dir

            result = subprocess.run(
                [shell_exe] + shell_args if venv_dir else [shell_exe, info_tool, target_path],
                capture_output=True, text=True, env=env, check=False
            )
            
            if result.returncode != 0:
                error_msg = result.stderr if result.stderr.strip() else "Extraction tool failed with no error message."
                self.log(f"Error: Extraction failed with exit code {result.returncode}")
                self.log(error_msg)
                QMessageBox.critical(self, "Extraction Error", 
                    f"Failed to extract options from target script using {os.path.basename(info_tool)}.\n\n"
                    f"Exit code: {result.returncode}\n\n"
                    f"Error output:\n{error_msg}")
                return

            try:
                options_data = json.loads(result.stdout)
                self.generate_ui_from_json(options_data)
            except json.JSONDecodeError:
                self.log("Error: Extraction tool returned invalid JSON.")
                self.log(result.stdout)
                QMessageBox.critical(self, "Extraction Error", "Extraction tool returned invalid JSON. See log for details.")
                return

        except Exception as e:
            self.log(f"Exception during target parsing: {str(e)}")
            QMessageBox.critical(self, "Parsing Error", f"Failed to run target extraction: {e}")

    def generate_ui_from_json(self, options_data):
        self.tabs.clear()
        self.target_args = {}

        # Group arguments by their group name
        groups = {}
        for opt in options_data:
            group_name = opt.get("group", "Arguments")
            if group_name not in groups:
                groups[group_name] = []
            groups[group_name].append(opt)

        for group_name, options in groups.items():
            tab = QWidget()
            tab_layout = QVBoxLayout(tab)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll_content = QWidget()
            flow = FlowLayout(scroll_content)

            for opt in options:
                flags = opt.get("flags", [])
                if not flags:
                    continue
                
                long_name = next((f for f in flags if f.startswith('--')), flags[0])
                clean_name = long_name.lstrip('-')
                
                if clean_name in ['help', 'h']:
                    continue

                help_desc = opt.get("help") or ""
                default_val = opt.get("default")
                choices = opt.get("choices")
                opt_type = opt.get("type", "string")

                # Handle null/None defaults for bools
                if opt_type == "bool" and default_val is None:
                    default_val = False

                # Special cases for targets where default is missing in help/metadata but known in the project
                if default_val is None or default_val == "":
                    if long_name == '--board' and 'xtrx' in self.current_target_path.lower():
                        default_val = 'limesdr'
                    elif long_name == '--cpu-type':
                        if 'xtrx' in self.current_target_path.lower():
                            default_val = 'vexriscv_smp'
                        else:
                            default_val = 'vexriscv'
                    elif long_name == '--cable':
                        default_val = 'ft2232'

                widget = None
                if choices:
                    widget = QComboBox()
                    widget.addItems([str(c) for c in choices])
                    if default_val is not None:
                        widget.setCurrentText(str(default_val))
                    widget.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Fixed)
                    widget.setMaximumWidth(200)
                    self.target_args[long_name] = ('choice', widget)
                elif opt_type == "bool":
                    widget = QCheckBox()
                    if default_val is True:
                        widget.setChecked(True)
                    widget.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
                    self.target_args[long_name] = ('bool', widget)
                else:
                    widget = QLineEdit()
                    if default_val is not None:
                        widget.setText(str(default_val))
                    widget.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Fixed)
                    widget.setMaximumWidth(200)
                    self.target_args[long_name] = ('text', widget)

                label = QLabel(clean_name.replace('-', ' ').title())
                label.setToolTip(help_desc)
                label.setStyleSheet("font-weight: bold; color: #333;")

                container = QFrame()
                container.setFrameShape(QFrame.Shape.StyledPanel)
                container.setStyleSheet("""
                    QFrame {
                        background-color: #f9f9f9;
                        border: 1px solid #ddd;
                        border-radius: 4px;
                        padding: 5px;
                    }
                    QFrame:hover {
                        background-color: #f0f0f0;
                        border-color: #ccc;
                    }
                """)
                container_layout = QVBoxLayout(container)
                container_layout.setContentsMargins(8, 8, 8, 8)
                container_layout.setSpacing(5)
                container_layout.addWidget(label)
                container_layout.addWidget(widget)
                container.setFixedWidth(220)
                flow.addWidget(container)

            scroll.setWidget(scroll_content)
            tab_layout.addWidget(scroll)
            self.tabs.addTab(tab, group_name.rstrip(':'))

    def show_settings(self):
        # Simple dialog to add toolchain environment scripts (e.g. source /opt/intelFPGA/...)
        msg = "Enter paths to toolchain environment scripts (one per line).\nThese will be used to setup the environment before building."
        text, ok = QInputDialog.getMultiLineText(self, "Toolchain Settings", msg, "\n".join(self.toolchain_env_scripts))
        if ok:
            self.toolchain_env_scripts = [l.strip() for l in text.split('\n') if l.strip()]

    def run_setup_litex(self):
        script = os.path.join(os.getcwd(), "setup_litex.sh")
        if not os.path.exists(script):
            QMessageBox.critical(self, "Error", f"setup_litex.sh not found at {script}")
            return
        
        self.terminal.clear()
        self.log("Running setup_litex.sh --install...")
        self.build_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.setup_button.setEnabled(False)
        
        # On Linux/macOS, we run it with bash.
        self.process.start("bash", [script, "--install"])

    def start_build(self):
        if not self.venv_python:
            QMessageBox.critical(self, "No VENV", "Virtual environment not found. Build prohibited. Please run setup_litex.sh first.")
            return

        if not self.current_target_path:
            QMessageBox.warning(self, "No Target", "Please select a target first.")
            return

        cmd = [self.venv_python, self.current_target_path]
        
        for arg_name, (arg_type, widget) in self.target_args.items():
            if arg_type == 'bool':
                if widget.isChecked():
                    cmd.append(arg_name)
                else:
                    # Note: If a boolean defaults to True, LiteX usually provides a negation flag 
                    # like --no-<name>. However, since we parse help dynamically, we don't 
                    # always know the negation flag name. 
                    # For now, we omit the flag, assuming default-false is standard.
                    # If default-true is common, specific negation logic would be needed.
                    pass
            elif arg_type == 'choice':
                cmd.append(arg_name)
                cmd.append(widget.currentText())
            elif arg_type == 'text':
                val = widget.text().strip()
                if val:
                    cmd.append(arg_name)
                    cmd.append(val)

        self.terminal.clear()
        self.log(f"Executing: {' '.join(cmd)}")
        
        # Prepare environment
        q_env = QProcessEnvironment.systemEnvironment()
        q_env.insert("PYTHONPATH", os.getcwd())
        # Set VIRTUAL_ENV to point to our .venv
        venv_dir = os.path.dirname(os.path.dirname(self.venv_python))
        q_env.insert("VIRTUAL_ENV", venv_dir)
        
        self.process.setProcessEnvironment(q_env)

        # Build activation and execution command
        if platform.system() == "Windows":
            activate_cmd = f"call {os.path.join(venv_dir, 'Scripts', 'activate.bat')}"
            python_exe = "python"
        else:
            activate_cmd = f"source {os.path.join(venv_dir, 'bin', 'activate')}"
            python_exe = "python3"

        # Use the activated python instead of absolute path
        cmd[0] = python_exe
        
        if self.toolchain_env_scripts:
            if platform.system() == "Windows":
                shell_cmd = " && ".join([f"call {s}" for s in self.toolchain_env_scripts])
                full_cmd_str = f"{activate_cmd} && {shell_cmd} && {' '.join(cmd)}"
                self.process.start("cmd", ["/c", full_cmd_str])
            else:
                shell_cmd = " && ".join([f"source {s}" for s in self.toolchain_env_scripts])
                full_cmd_str = f"{activate_cmd} && {shell_cmd} && {' '.join(cmd)}"
                self.process.start("bash", ["-c", full_cmd_str])
        else:
            if platform.system() == "Windows":
                full_cmd_str = f"{activate_cmd} && {' '.join(cmd)}"
                self.process.start("cmd", ["/c", full_cmd_str])
            else:
                full_cmd_str = f"{activate_cmd} && {' '.join(cmd)}"
                self.process.start("bash", ["-c", full_cmd_str])

        self.build_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def stop_build(self):
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.log("\nStopping build process...")
            self.process.terminate()
            if not self.process.waitForFinished(3000):
                self.process.kill()

    def on_stdout(self):
        data = self.process.readAllStandardOutput().data().decode(errors='replace')
        cursor = self.terminal.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        cursor.insertText(data)
        self.terminal.setTextCursor(cursor)
        self.terminal.verticalScrollBar().setValue(self.terminal.verticalScrollBar().maximum())

    def on_stderr(self):
        data = self.process.readAllStandardError().data().decode(errors='replace')
        cursor = self.terminal.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        cursor.insertText(data)
        self.terminal.setTextCursor(cursor)
        self.terminal.verticalScrollBar().setValue(self.terminal.verticalScrollBar().maximum())

    def on_process_finished(self, exit_code, exit_status):
        self.log(f"\nProcess finished with exit code {exit_code}")
        self.build_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.setup_button.setEnabled(not self.find_venv_python())
        
        # Refresh venv if we just ran setup
        if not self.venv_python:
            self.venv_python = self.find_venv_python()
            if self.venv_python:
                self.on_target_selected()

    def log(self, message):
        cursor = self.terminal.textCursor()
        cursor.movePosition(cursor.MoveOperation.End)
        cursor.insertText(message + '\n')
        self.terminal.setTextCursor(cursor)
        self.terminal.verticalScrollBar().setValue(self.terminal.verticalScrollBar().maximum())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LiteXBuildGUI()
    window.show()
    sys.exit(app.exec())
