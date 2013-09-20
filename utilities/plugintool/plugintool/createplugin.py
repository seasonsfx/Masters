#!/usr/bin/python
import os
import sys
import shutil
import plugintool
from mako.template import Template

here = os.path.dirname(os.path.abspath(__file__))

def usage(argv):
    cmd = os.path.basename(argv[0])
    print('usage: %s <PluginName>\n'
          '(example: "%s MyPlugin")' % (cmd, cmd))
    sys.exit(1)


def main(argv=sys.argv):
    if len(argv) != 2:
        usage(argv)
    
    templates = [
        {"in": "template/template.cpp", "out": "${lower_name}/${lower_name}.cpp"},
        {"in": "template/template.h", "out": "${lower_name}/${lower_name}.h"},
        {"in": "template/template.qrc", "out": "${lower_name}/${lower_name}.qrc"},
        {"in": "template/template.json", "out": "${lower_name}/${lower_name}.json"},
        {"in": "template/CMakeLists.txt", "out": "${lower_name}/CMakeLists.txt"},
    ]

    name = argv[1].strip("'\"")
    safename = name.replace(" ", "")

    context = {
        "name" : name,
        "safename" : safename,
        "lower_name" : safename.lower(),
        "upper_name" : safename.upper(),
        "camel_name" : "".join(n[0].upper() + n[1:] for n in name.split()),
        "icon" : safename.lower() + ".png",
    }

    newpath = context["lower_name"]
    print safename
    if not os.path.exists(newpath): os.makedirs(newpath)

    iconsrc = os.path.abspath(os.path.join(here, "template/template.png"))
    icondest = Template("${lower_name}/${icon}").render(**context)
    shutil.copy2(iconsrc, icondest)

    for t in templates:
        tpath = os.path.abspath(os.path.join(here, t["in"]))
        tmpl = Template(filename=tpath)
        filename = Template(t["out"]).render(**context)
        outfile = open(filename, 'w')
        content = tmpl.render(**context)
        outfile.write(content)
        outfile.close()