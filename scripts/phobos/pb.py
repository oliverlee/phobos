import contextlib
import os
import re
import numpy as np
from google.protobuf.internal.decoder import _DecodeVarint32
from google.protobuf.descriptor import FieldDescriptor


class MissingDataError(Exception):
    pass


class ExtraDataError(Exception):
    pass


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
    if (length + position) > len(data):
        raise MissingDataError
    if (length + position) < len(data):
        raise ExtraDataError

    message.ParseFromString(data[position:position + length])
    return message


__np_dtype_map = dict([
    (FieldDescriptor.CPPTYPE_INT32, np.int32),
    (FieldDescriptor.CPPTYPE_INT64, np.int64),
    (FieldDescriptor.CPPTYPE_UINT32, np.uint32),
    (FieldDescriptor.CPPTYPE_UINT64, np.uint64),
    (FieldDescriptor.CPPTYPE_DOUBLE, np.float64),
    (FieldDescriptor.CPPTYPE_FLOAT, np.float32),
    (FieldDescriptor.CPPTYPE_BOOL, np.bool_),
    #(FieldDescriptor.CPPTYPE_ENUM, None),          not handled
    (FieldDescriptor.CPPTYPE_STRING, np.uint8),
    #(FieldDescriptor.CPPTYPE_MESSAGE, None),       handled by recursion
])


def __is_repeating_field(field_descriptor):
    return ((field_descriptor.label == FieldDescriptor.LABEL_REPEATED) or
            (field_descriptor.cpp_type == FieldDescriptor.CPPTYPE_STRING))


def __has_only_single_repeating_field(field_descriptor):
    try:
        fields = field_descriptor.message_type.fields
    except AttributeError:
        return False
    else:
        if ((len(fields) == 1) and (__is_repeating_field(fields[0]))):
            return True
    return False


def get_np_dtype(message_type_descriptor, max_repeated=None):
    dtype = []
    for field_descriptor in message_type_descriptor.fields:
        shape = 1
        try:
            scalar_type = __np_dtype_map[field_descriptor.cpp_type]
            if __is_repeating_field(field_descriptor):
                # If the field is repeated or bytes/string type, the max_count
                # or max_size _must_ be defined so that an upper bound for
                # memory usage is known.
                shape = max_repeated[field_descriptor.full_name]
        except KeyError:
            scalar_type = get_np_dtype(field_descriptor.message_type,
                                       max_repeated)
            # If we have a single repeated field (such as for matrix message
            # types), don't create a record with a single element to make
            # access a bit easier.
            if __has_only_single_repeating_field(field_descriptor):
                # Here is where we remove a level of nesting in the numpy dtype
                # TODO: Change shape to a tuple so we have a matrix/vector?
                #       Eigen stores elements in column-major order so we must
                #       also do the same here.
                v  = list(scalar_type.fields.values())[0][0]
                scalar_type, shape = v.subdtype
        finally:
            dtype.append((field_descriptor.name, scalar_type, shape))
    return np.dtype(dtype)


def set_record_from_message(record, message):
    for field_descriptor, value in message.ListFields():
        try:
            setattr(record, field_descriptor.name, value)
        except TypeError:
            if __has_only_single_repeating_field(field_descriptor):
                # As we simplify the numpy dtype, we also need to skip a level
                # when copying a protobuf message to a numpy record.
                fd, value = value.ListFields()[0]
                if fd.cpp_type == FieldDescriptor.CPPTYPE_STRING:
                    # numpy doesn't have a string type so we use the byte type
                    # defined in the map
                    scalar_type = __np_dtype_map[fd.cpp_type]
                    value = np.fromstring(value, scalar_type)
                setattr(record, field_descriptor.name, value)
            else:
                subrecord = getattr(record, field_descriptor.name)
                set_record_from_message(subrecord, value)


def make_max_repeated_dict(options_file):
    # TODO: probably allow multiple option files
    pattern = re.compile(r'(\w+\.\w+)\s+max_(?:size|count):(\d+)')
    max_repeated = dict()
    with open(options_file, 'r') as options:
        for line in options:
            match = pattern.match(line)
            if match:
                max_repeated[match.group(1)] = int(match.group(2))
    return max_repeated


if __name__ == '__main__':
    import sys
    import_modules(sys.argv[1:])
