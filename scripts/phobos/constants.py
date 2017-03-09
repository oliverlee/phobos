import os
import re
import numexpr as ne

def parse_constants():
    local_dir = os.path.dirname(os.path.realpath(__file__))
    constants_file = os.path.join(local_dir, os.path.pardir, os.path.pardir,
                                  'projects', 'inc', 'saconfig.h')
    class Object(object):
        pass

    sa = Object()
    failed_matches = []
    with open(constants_file, 'r') as f:
        pattern = re.compile(
                r'constexpr (float|dacsample_t|adcsample_t) ([A-Z_]+) = (.*);')

        for line in f:
            match = pattern.match(line)
            if match:
                scalar_type, name, value = match.groups()
                value = value.replace('f', '')
                try:
                    ev = ne.evaluate(value)
                except KeyError:
                    # try match again later as it may refer to another entry
                    failed_matches.append(match.groups())
                    continue
                else:
                    print(ev)
                    if scalar_type == 'float':
                        v = float(ev)
                    else:
                        v = int(ev)
                    setattr(sa, name, v)

        # only do one pass of failed matches
        for _, name, value in failed_matches:
            try:
                v = getattr(sa, name)
                setattr(sa, name, v)
            except AttributeError:
                pass
    return sa

sa = parse_constants()
