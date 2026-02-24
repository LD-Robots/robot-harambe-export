#!/usr/bin/env python3
"""Split robot_harambe.urdf into organized xacro files."""

import xml.etree.ElementTree as ET
from pathlib import Path

URDF_PATH = Path(__file__).parent / "robot_harambe.urdf"
SPLIT_DIR = Path(__file__).parent / "split"

XACRO_NS = "http://www.ros.org/wiki/xacro"
XACRO_HEADER = '<?xml version="1.0" ?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
XACRO_FOOTER = "</robot>\n"

# Define which links/joints belong to which group
GROUPS = {
    "base": {
        "links": ["urdf_base"],
        "joints": [],
    },
    "torso": {
        "links": ["urdf_simplified_torso_sub_assembly"],
        "joints": ["waist_yaw_joint"],
    },
    "left_arm": {
        "links": [
            "urdf_l_shoulder_assembly",
            "urdf_l_upper_arm_assembly",
            "urdf_l_elbow_assembly",
            "urdf_l_upper_forearm_assembly",
            "urdf_l_lower_forearm_assembly",
            "urdf_l_wrist_assembly",
        ],
        "joints": [
            "left_shoulder_pitch_joint_X6",
            "left_shoulder_roll_joint_X6",
            "left_shoulder_yaw_joint_X4",
            "left_elbow_pitch_joint_X6",
            "left_wrist_yaw_joint_X4",
            "left_wrist_roll_joint_X4",
        ],
    },
    "right_arm": {
        "links": [
            "urdf_r_shoulder_assembly",
            "urdf_r_upper_arm_assembly",
            "urdf_r_elbow_assembly",
            "urdf_r_upper_forearm_assembly",
            "urdf_r_lower_forearm_assembly",
            "urdf_r_wrist_assembly",
        ],
        "joints": [
            "right_shoulder_pitch_joint_X6",
            "right_shoulder_roll_joint_X6",
            "right_shoulder_yaw_joint_X4",
            "right_elbow_pitch_joint_X6",
            "right_wrist_yaw_joint_X4",
            "right_wrist_roll_joint_X4",
        ],
    },
    "left_leg": {
        "links": [
            "urdf_hip_assembly",
            "urdf_l_upper_femur_assembly",
            "urdf_lower_femur_assembly",
            "urdf_l_tibia_assembly",
            "ankle_assembly",
            "ankle_assembly_2",
        ],
        "joints": [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_pitch_joint_2",
        ],
    },
    "right_leg": {
        "links": [
            "urdf_hip_assembly_2",
            "urdf_r_upper_femur_assemby",
            "urdf_lower_femur_assembly_2",
            "urdf_r_tibia_assembly",
            "ankle_assembly_3",
            "ankle_assembly_4",
        ],
        "joints": [
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_pitch_joint_2",
        ],
    },
}


def element_to_string(elem, indent="  "):
    """Convert an XML element to a nicely indented string."""
    lines = []
    _serialize(elem, lines, indent, level=1)
    return "\n".join(lines) + "\n"


def _serialize(elem, lines, indent, level):
    """Recursively serialize XML element with proper indentation."""
    prefix = indent * level
    tag = elem.tag
    attribs = "".join(f' {k}="{v}"' for k, v in elem.attrib.items())

    # Get text and tail
    text = elem.text
    children = list(elem)

    if not children and (text is None or text.strip() == ""):
        lines.append(f"{prefix}<{tag}{attribs}/>")
    elif not children:
        lines.append(f"{prefix}<{tag}{attribs}>{text.strip()}</{tag}>")
    else:
        lines.append(f"{prefix}<{tag}{attribs}>")
        for child in children:
            # Add comment if there was one before this child
            _serialize(child, lines, indent, level + 1)
        lines.append(f"{prefix}</{tag}>")


def parse_with_comments(filepath):
    """Parse URDF preserving comment information by reading raw file."""
    tree = ET.parse(filepath)
    root = tree.getroot()

    # Also read raw lines to extract comments
    with open(filepath) as f:
        raw_content = f.read()

    return root, raw_content


def extract_element_with_comments(raw_content, tag, name_attr):
    """Extract a complete element including its preceding comment from raw content."""
    import re

    # Find the comment + element pattern
    # Comments look like: <!-- Link urdf_base --> or <!-- Joint from ... -->
    if tag == "link":
        pattern = rf'(  <!-- Link {re.escape(name_attr)} -->\n)?  <link name="{re.escape(name_attr)}">'
    else:
        pattern = rf'(  <!-- Joint from [^>]+ -->\n)?  <joint name="{re.escape(name_attr)}"'

    match = re.search(pattern, raw_content)
    if not match:
        print(f"WARNING: Could not find {tag} name={name_attr}")
        return None

    start = match.start()

    # Find the closing tag
    elem_start = raw_content.index(f"<{tag} name=\"{name_attr}\"", start)
    if tag == "link":
        close_tag = "</link>"
    else:
        close_tag = "</joint>"

    close_idx = raw_content.index(close_tag, elem_start) + len(close_tag)
    return raw_content[start:close_idx]


def write_xacro_file(filepath, elements_content):
    """Write a xacro file with the given element content strings."""
    with open(filepath, "w") as f:
        f.write(XACRO_HEADER)
        for content in elements_content:
            f.write("\n")
            f.write(content)
            f.write("\n")
        f.write("\n")
        f.write(XACRO_FOOTER)


def main():
    root, raw_content = parse_with_comments(URDF_PATH)

    for group_name, group_data in GROUPS.items():
        group_dir = SPLIT_DIR / group_name

        # Extract links
        if group_data["links"]:
            link_contents = []
            for link_name in group_data["links"]:
                content = extract_element_with_comments(raw_content, "link", link_name)
                if content:
                    link_contents.append(content)
            if link_contents:
                write_xacro_file(
                    group_dir / f"{group_name}_links.urdf.xacro", link_contents
                )
                print(f"  Written {group_name}_links.urdf.xacro ({len(link_contents)} links)")

        # Extract joints
        if group_data["joints"]:
            joint_contents = []
            for joint_name in group_data["joints"]:
                content = extract_element_with_comments(raw_content, "joint", joint_name)
                if content:
                    joint_contents.append(content)
            if joint_contents:
                write_xacro_file(
                    group_dir / f"{group_name}_joints.urdf.xacro", joint_contents
                )
                print(f"  Written {group_name}_joints.urdf.xacro ({len(joint_contents)} joints)")

    # Write the main xacro that includes everything
    main_xacro = SPLIT_DIR / "robot_harambe.urdf.xacro"
    with open(main_xacro, "w") as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="robot_harambe" xmlns:xacro="http://www.ros.org/wiki/xacro">\n')
        f.write("\n")
        f.write("  <!-- Base -->\n")
        f.write('  <xacro:include filename="base/base_links.urdf.xacro"/>\n')
        f.write("\n")
        f.write("  <!-- Torso -->\n")
        f.write('  <xacro:include filename="torso/torso_links.urdf.xacro"/>\n')
        f.write('  <xacro:include filename="torso/torso_joints.urdf.xacro"/>\n')
        f.write("\n")
        f.write("  <!-- Left Arm -->\n")
        f.write('  <xacro:include filename="left_arm/left_arm_links.urdf.xacro"/>\n')
        f.write('  <xacro:include filename="left_arm/left_arm_joints.urdf.xacro"/>\n')
        f.write("\n")
        f.write("  <!-- Right Arm -->\n")
        f.write('  <xacro:include filename="right_arm/right_arm_links.urdf.xacro"/>\n')
        f.write('  <xacro:include filename="right_arm/right_arm_joints.urdf.xacro"/>\n')
        f.write("\n")
        f.write("  <!-- Left Leg -->\n")
        f.write('  <xacro:include filename="left_leg/left_leg_links.urdf.xacro"/>\n')
        f.write('  <xacro:include filename="left_leg/left_leg_joints.urdf.xacro"/>\n')
        f.write("\n")
        f.write("  <!-- Right Leg -->\n")
        f.write('  <xacro:include filename="right_leg/right_leg_links.urdf.xacro"/>\n')
        f.write('  <xacro:include filename="right_leg/right_leg_joints.urdf.xacro"/>\n')
        f.write("\n")
        f.write("</robot>\n")
    print(f"\n  Written main: robot_harambe.urdf.xacro")

    # Verify completeness
    all_links = set()
    all_joints = set()
    for g in GROUPS.values():
        all_links.update(g["links"])
        all_joints.update(g["joints"])

    urdf_links = {elem.get("name") for elem in root.findall("link")}
    urdf_joints = {elem.get("name") for elem in root.findall("joint")}

    missing_links = urdf_links - all_links
    missing_joints = urdf_joints - all_joints
    extra_links = all_links - urdf_links
    extra_joints = all_joints - urdf_joints

    if missing_links:
        print(f"\n  WARNING: Missing links: {missing_links}")
    if missing_joints:
        print(f"\n  WARNING: Missing joints: {missing_joints}")
    if extra_links:
        print(f"\n  WARNING: Extra links (not in URDF): {extra_links}")
    if extra_joints:
        print(f"\n  WARNING: Extra joints (not in URDF): {extra_joints}")
    if not missing_links and not missing_joints and not extra_links and not extra_joints:
        print("\n  All links and joints accounted for!")


if __name__ == "__main__":
    main()
