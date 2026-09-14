#!/usr/bin/env python3
"""Apply a guarded, observation-only patch to the four pinned production files."""
import hashlib
from pathlib import Path

PACKAGE=Path(__file__).resolve().parents[2]


def replace_one(text, before, after):
    if text.count(before)!=1:
        raise ValueError('Expected one reviewed source anchor: '+before[:100])
    return text.replace(before,after)


def main():
    edits={
      'include/unicycle_ugv_controller/common/types.h':(
        '355d3d120000a9e73ca5b89ffe858044d763170d',[
        ('#include <geometry_msgs/Twist.h>', '#include "unicycle_ugv_controller/common/flatness_diagnostic.h"\n\n#include <geometry_msgs/Twist.h>'),
        ('    bool valid{false};\n};\n\nstruct ResetTarget', '    bool valid{false};\n    FlatnessDiagnostic flatness_diagnostic{};\n};\n\nstruct ResetTarget')]),
      'src/state_machine/custom1_state.cpp':(
        'c1e3f8c0f6bf9993b7f44a3ebd0a302880166930',[
        ('    const FlatnessCommandOutput output =\n', '    const double command_speed_before = body_speed_;\n    const FlatnessCommandOutput output =\n'),
        ('    command.angular_speed = output.angular_speed;\n    command.valid = true;',
         '    command.angular_speed = output.angular_speed;\n    command.valid = true;\n    command.flatness_diagnostic = makeFlatnessDiagnostic(\n        now, dt, command_speed_before, snapshot, lifted, output);')]),
      'include/unicycle_ugv_controller/output/cmd_vel_output_consumer.h':(
        'a7e6552a48f37786cf8e5ebe5b1040938de82236',[
        ('    ros::Publisher cmd_vel_pub_;', '    ros::Publisher cmd_vel_pub_;\n    ros::Publisher flatness_diagnostic_pub_;')]),
      'src/output/cmd_vel_output_consumer.cpp':(
        'f62470adee5b9dcb8ad74db6b22c4e27b6753719',[
        ('#include <utility>', '#include <utility>\n#include <std_msgs/Float64MultiArray.h>'),
        ('    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);',
         '    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);\n    flatness_diagnostic_pub_ = nh.advertise<std_msgs::Float64MultiArray>(\n        cmd_vel_topic + "/flatness_diagnostic", 1);'),
        ('        const auto command = makeTwist(controller_.command());\n        cmd_vel_pub_.publish(command);',
         '        const auto snapshot = controller_.command();\n        const auto command = makeTwist(snapshot);\n        const double publication_time = ros::Time::now().toSec();\n        cmd_vel_pub_.publish(command);\n        const auto diagnostic = atFlatnessPublication(snapshot.flatness_diagnostic,\n            publication_time, command.linear.x, command.angular.z);\n        if (diagnostic.valid) {\n            std_msgs::Float64MultiArray message;\n            message.layout.dim.resize(1);\n            message.layout.dim[0].label = flatnessDiagnosticSchema();\n            message.layout.dim[0].size = FlatnessDiagnostic::Count;\n            message.layout.dim[0].stride = FlatnessDiagnostic::Count;\n            message.data.assign(diagnostic.values.begin(), diagnostic.values.end());\n            flatness_diagnostic_pub_.publish(message);\n        }')])
    }
    replacements=[]
    for relative,(expected,changes) in edits.items():
        path=PACKAGE/relative;raw=path.read_bytes();text=raw.decode()
        if all(after in text for _,after in changes):
            continue
        blob=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
        if blob!=expected:
            raise ValueError('Refusing unreviewed production source '+relative+': '+blob)
        for before,after in changes:text=replace_one(text,before,after)
        replacements.append((path,text))
    for path,text in replacements:path.write_text(text)
    print('Patched '+str(len(replacements))+' observation-only source files; flatness law unchanged')


if __name__=='__main__':main()
