import contextlib
import os
from google.protobuf.internal.decoder import _DecodeVarint32


def import_modules(proto_paths):
    if not proto_paths:
        return

    generated_files = []
    for pp in proto_paths:
        dirname, filename = os.path.split(os.path.abspath(pp))
        output_dir = os.getcwd()
        file_basename, _ = os.path.splitext(filename)
        os.system('protoc --proto_path={0} --python_out={1} {0}/{2}'.format(
            dirname, output_dir, filename))
        generated_file = os.path.join(output_dir, file_basename + '_pb2.py')
        assert os.path.isfile(generated_file)
        generated_files.append(generated_file)

    imports = []
    for g in generated_files:
        name, _ = os.path.splitext(os.path.basename(g))
        imports.append(__import__(name))
        # after importing module, delete generated python protobuf descriptor
        # and compiled bytecode if it exists
        os.remove(g)
        with contextlib.suppress(FileNotFoundError):
            os.remove(g + 'c')
    return imports


def decode_delimited(message, data):
    length, position = _DecodeVarint32(data, 0)
    message.ParseFromString(data[position:position + length])
    return message


if __name__ == '__main__':
    import sys
    import_modules(sys.argv[1:])
