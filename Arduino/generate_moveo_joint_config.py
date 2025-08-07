import xml.etree.ElementTree as ET

INPUT_XACRO = "ros2_ws/iteration_5/urdf/moveo.ros2_control.xacro"
OUTPUT_HEADER = "Arduino/config.h"

def parse_joint_params(joint_element):
    name = joint_element.attrib.get("name", "Unknown")
    params = {param.attrib["name"]: param.text for param in joint_element.findall("param")}
    return {
        "name": name,
        "id": int(params["id"]),
        "pin_pulse": params["pin_pulse"],
        "pin_dir": params["pin_dir"],
        "direction": int(params["direction"]),
        "steps_per_revolution": int(params["steps_per_revolution"])
    }

def parse_hardware_params(hardware_element):
    return {param.attrib["name"]: param.text for param in hardware_element.findall("param")}

def generate_header(joints, hw_params):
    lines = []
    lines.append("// Auto-generated file from ros2_control.xacro")
    lines.append("#pragma once")
    lines.append("#include \"header.h\"\n#include <Arduino.h>\n")
    # Add hardware parameters as macros
    if "baud_rate" in hw_params:
        lines.append(f"#define BAUD_RATE {hw_params['baud_rate']}")
    if "serial_device" in hw_params:
        lines.append(f'// Serial device: {hw_params["serial_device"]}')
    if "i2c_bus" in hw_params:
        lines.append(f'// I2C bus: {hw_params["i2c_bus"]}')
    if "timeout" in hw_params:
        lines.append(f"#define SERIAL_TIMEOUT_MS {hw_params['timeout']}")
    
    lines.append(f"\n#define NUM_JOINTS {len(joints)}\n")

    # Joints array
    lines.append("moveoJoint joints[NUM_JOINTS] = {")
    for joint in joints:
        lines.append(
            f"  moveoJoint({joint['id']},{joint['steps_per_revolution']}, {joint['direction']}, "
            f"{joint['pin_pulse']}, {joint['pin_dir']}), // {joint['name']}"
        )
    lines.append("};\n")

    return "\n".join(lines)

def main():
    tree = ET.parse(INPUT_XACRO)
    root = tree.getroot()
    
    ros2_control = root.find(".//ros2_control")
    if ros2_control is None:
        raise RuntimeError("No <ros2_control> tag found in Xacro.")

    hardware_element = ros2_control.find("hardware")
    hw_params = parse_hardware_params(hardware_element)

    joint_elements = ros2_control.findall("joint")
    joints = [parse_joint_params(joint) for joint in joint_elements]
    joints.sort(key=lambda j: j["id"])  # Sort by joint ID

    header_code = generate_header(joints, hw_params)

    with open(OUTPUT_HEADER, "w") as f:
        f.write(header_code)

    print(f"✅ Generated {OUTPUT_HEADER} with {len(joints)} joints and hardware config.")

if __name__ == "__main__":
    main()