#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import jinja2


def generate_source(filename, output_directory, template_dict):
    template = template_setup(filename)
    filepath, in_ext = os.path.splitext(filename)
    assert in_ext == '.in', 'Invalid template file: {}'.format(filename)

    basename = os.path.basename(filepath)
    output_filename = os.path.abspath(os.path.join(
        output_directory, basename))

    with open(output_filename, 'w') as f:
        f.write(template.render(template_dict))
        f.write('\n')


def template_setup(filename):
    dirname, basename = os.path.split(os.path.abspath(filename))

    template_loader = jinja2.FileSystemLoader(dirname)
    template_env = jinja2.Environment(loader=template_loader)
    return template_env.get_template(basename);


class Field(object):
    def __init__(self, name, pbtype, tag, configurable):
        self.name = name
        self.pbtype = pbtype
        self.tag = tag
        self.configurable = configurable

    def __str__(self):
        return '{}, {}, {}, {}'.format(self.name,
                                       self.pbtype,
                                       self.tag,
                                       self.configurable)

if __name__ == "__main__":
    usage = 'generate protobuf schema files\n'
    usage += '{0} <template_file> <output_directory>'.format(__file__)
    if len(sys.argv) < 2:
        print(usage)
        sys.exit(1)

    # Configure pbRxMaster/pbSmallMessageGroup/pbTxMaster
    field = [
        # These fields are transmitted (somewhat) frequently
        Field('v',                              'float',            3, True),
        Field('dt',                             'float',            4, True),
        Field('controller_feedback_gains',      'pbFeedbackGains',  5, True),
        Field('controller_feedforward_inertia', 'float',            6, True),

        Field('pose', 'pbPose', 7, False),

        # Starting from 16 are less frequent fields
        # Configurable fields in this section are ignored while running
        Field('M',                      'pbSecondOrderMatrix',  16, True),
        Field('C1',                     'pbSecondOrderMatrix',  17, True),
        Field('K0',                     'pbSecondOrderMatrix',  18, True),
        Field('K2',                     'pbSecondOrderMatrix',  19, True),
        Field('trail',                  'float',                20, True),
        Field('steer_axis_tilt',        'float',                21, True),
        Field('rear_wheel_radius',      'float',                22, True),
        Field('front_wheel_radius',     'float',                23, True),

        Field('build_config', 'pbBuildConfig', 24, False),
    ]

    for f in field:
        assert f.tag != 1, "reserved for start / timestamp"
        assert f.tag != 2, "reserved for stop / simulation"

    template_dict = {'field': field}
    generate_source(sys.argv[1], sys.argv[2], template_dict)
