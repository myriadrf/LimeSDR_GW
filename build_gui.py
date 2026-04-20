import sys
import os
import re
import subprocess
import platform
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
        target_path = self.target_combo.currentData()
        if not target_path:
            return
        
        if not self.venv_python:
            QMessageBox.warning(self, "No VENV", "Virtual environment not found. Please run setup_litex.sh first.")
            return

        self.log(f"Parsing target: {os.path.basename(target_path)}")
        self.parse_target_help(target_path)

    def parse_target_help(self, target_path):
        # Run python target.py --help
        try:
            env = os.environ.copy()
            env["PYTHONPATH"] = os.getcwd()
            env["COLUMNS"] = "999"  # Prevent argparse from line-wrapping
            
            result = subprocess.run(
                [self.venv_python, target_path, "--help"],
                capture_output=True, text=True, env=env, check=False
            )
            
            if result.returncode != 0:
                error_msg = result.stderr if result.stderr.strip() else "Target script failed with no error message."
                self.log(f"Error: Target script failed with exit code {result.returncode}")
                self.log(error_msg)
                QMessageBox.warning(self, "Target Error", f"Failed to get help from target script (Exit {result.returncode}):\n\n{error_msg}")
                return

            if not result.stdout.strip():
                error_msg = result.stderr if result.stderr.strip() else "Target script returned empty output."
                self.log("Error: Target script returned no help output.")
                if error_msg:
                    self.log(error_msg)
                QMessageBox.warning(self, "Target Error", f"Target script returned no help output.\n\n{error_msg}")
                return

            self.generate_ui_from_help(result.stdout)
        except Exception as e:
            self.log(f"Exception during target parsing: {str(e)}")
            QMessageBox.critical(self, "Parsing Error", f"Failed to run target help: {e}")

    def generate_ui_from_help(self, help_text):
        self.tabs.clear()
        self.target_args = {}

        # Pre-filter help text to remove noise
        if 'usage:' in help_text:
            help_text = help_text[help_text.find('usage:'):]
        
        filtered_lines = [line for line in help_text.splitlines() if not (line.strip().startswith('[') or line.strip().startswith('('))]
        clean_help = "\n".join(filtered_lines).strip()

        # Split help text into sections based on groups
        # Groups look like "options:", "SoC options:", "Builder options:", etc.
        sections = re.split(r'\n(?=[A-Za-z0-9\s]+:)\n?', clean_help)
        
        # The first section is usually 'usage: ...'
        # Subsequent sections are groups
        for section in sections:
            section = section.strip()
            if not section or section.startswith("usage:"):
                continue
            
            lines = section.split('\n')
            group_name = lines[0].strip()
            group_content = "\n".join(lines[1:])
            
            tab = QWidget()
            tab_layout = QVBoxLayout(tab)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll_content = QWidget()
            
            # Use FlowLayout for responsive reflow
            flow = FlowLayout(scroll_content)
            
            # Regex to find arguments:
            # Matches: --arg-name, --choice {c1,c2}, --value VAL
            # We want to capture the argument name and its help text (including choices and default)
            # LiteX/Argparse help often uses indentation to group description with arg
            # We'll split by what looks like the start of a new argument
            group_content_clean = group_content.strip()
            args_blocks = re.split(r'\n(?=\s+-[a-zA-Z0-9-])', group_content_clean)
            
            arguments_found = 0
            for block in args_blocks:
                match = re.match(r'^\s*(-[a-zA-Z0-9-]+(?:,\s+--[a-zA-Z0-9-]+)?)(.*?)(?:\s{2,}|\n\s+)(.*)$', block, re.DOTALL)
                if not match:
                    continue
                
                arg_names_raw, meta, help_desc = match.groups()
                # Use the long name if available
                arg_names = [n.strip() for n in arg_names_raw.split(',')]
                long_name = next((n for n in arg_names if n.startswith('--')), arg_names[0])
                clean_name = long_name.lstrip('-')
                
                if clean_name == 'help' or clean_name == 'h':
                    continue

                arguments_found += 1
                widget = None
                
                # Check for choices: {choice1,choice2}
                choice_match = re.search(r'{(.*?)}', meta)
                if not choice_match:
                    # Sometimes choices are in help_desc but that's harder to parse reliably
                    choice_match = re.search(r'{(.*?)}', help_desc)

                # Check if it's a boolean flag (usually (default: False) or no meta)
                is_bool = "(default: False)" in help_desc or "(default: True)" in help_desc or not meta.strip()
                
                # Default value extraction
                default_match = re.search(r'\(default: (.*?)\)', help_desc)
                default_val = default_match.group(1) if default_match else ""

                if choice_match:
                    choices = [c.strip() for c in choice_match.group(1).split(',')]
                    widget = QComboBox()
                    widget.addItems(choices)
                    if default_val in choices:
                        widget.setCurrentText(default_val)
                    widget.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Fixed)
                    widget.setMaximumWidth(200)
                    self.target_args[long_name] = ('choice', widget)
                elif is_bool:
                    widget = QCheckBox()
                    if default_val.lower() == 'true':
                        widget.setChecked(True)
                    widget.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
                    self.target_args[long_name] = ('bool', widget)
                else:
                    widget = QLineEdit()
                    widget.setText(default_val)
                    widget.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Fixed)
                    widget.setMaximumWidth(200)
                    self.target_args[long_name] = ('text', widget)
                
                label = QLabel(clean_name.replace('-', ' ').title())
                label.setToolTip(help_desc.strip())
                label.setStyleSheet("font-weight: bold; color: #333;")
                
                # Card-like container (QFrame)
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
                
                # Ensure card doesn't stretch too much but has a base size
                container.setFixedWidth(220)
                
                flow.addWidget(container)

            if arguments_found > 0:
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
        target_path = self.target_combo.currentData()
        if not target_path or not self.venv_python:
            return

        cmd = [self.venv_python, target_path]
        
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
        
        if self.toolchain_env_scripts:
            # If scripts are provided, we wrap the execution in a shell for sourcing
            if platform.system() == "Windows":
                # Windows: cmd /c "call script1.bat && call script2.bat && python target.py ..."
                shell_cmd = " && ".join([f"call {s}" for s in self.toolchain_env_scripts])
                full_cmd_str = f"{shell_cmd} && {' '.join(cmd)}"
                self.process.start("cmd", ["/c", full_cmd_str])
            else:
                # Linux/macOS: bash -c "source script1.sh && source script2.sh && python target.py ..."
                shell_cmd = " && ".join([f"source {s}" for s in self.toolchain_env_scripts])
                full_cmd_str = f"{shell_cmd} && {' '.join(cmd)}"
                self.process.start("bash", ["-c", full_cmd_str])
        else:
            self.process.setProcessEnvironment(q_env)
            self.process.start(cmd[0], cmd[1:])

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
