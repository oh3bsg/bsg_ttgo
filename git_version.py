import sys
from io import StringIO
import subprocess

# Redirect stdout to an in-memory buffer
result_buffer = StringIO()
sys.stdout = result_buffer

subprocess.run(["git", "describe", "--abbrev=6", "--dirty", "--always", "--tags"])

# Get the captured output
git_version = result_buffer.getvalue()

# Restore stdout
sys.stdout = sys.__stdout__

'''
import sys
from io import StringIO

# Redirect stdout to an in-memory buffer
result_buffer = StringIO()
sys.stdout = result_buffer

# Print something
print("This is a test")

# Get the captured output
captured_output = result_buffer.getvalue()

# Restore stdout
sys.stdout = sys.__stdout__
'''
