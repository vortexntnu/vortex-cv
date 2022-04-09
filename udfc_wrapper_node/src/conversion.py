
import sys

if len(sys.argv) != 3:
    raise ValueError("Must specify input and output file")

src_name = sys.argv[1]
out_name = sys.argv[2]

with open(src_name, "r") as src_file:
    with open(out_name, "w") as dest_file:
        for i, line in enumerate(src_file.readlines()):
            if i > 0 and line[:-1] is not None:
                dest_file.write(f"const float {line[:-1]};\n")

