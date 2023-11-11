#import sys
#from io import StringIO
import subprocess

result = subprocess.run("git describe --abbrev=6 --dirty --always --tags", 
                        shell=True, 
                        capture_output = True,
                        text=True)

git_versio = result.stdout
print("GIT Version: ", git_versio)

f = open("RX_FSK/git_version.h", "w")

f.write("const char *version_name = \"rdzTTGOsonde\";\n")
f.write("const char *version_id = \"Multi_ch-")
f.write(git_versio[:-1])
f.write("\";\n")
f.write("const int SPIFFS_MAJOR=2;\n")
f.write("const int SPIFFS_MINOR=17;\n")

f.close()
