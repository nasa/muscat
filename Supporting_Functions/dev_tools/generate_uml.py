import re
import glob

def parse_matlab_class(file_path):
    with open(file_path, 'r') as f:
        content = f.read()

    class_name = re.search(r'classdef\s+(\w+)', content).group(1)
    properties = {'external': [], 'internal': []}
    methods = []
    
    # Parse properties
    prop_block = re.search(r'properties(.*?)end', content, re.DOTALL)
    if prop_block:
        prop_content = prop_block.group(1)
        current_section = None
        # Split on EXTERNAL/INTERNAL markers
        sections = re.split(r'%%\s*\[(EXTERNAL|INTERNAL)\]\s*', prop_content)
        
        for i, section in enumerate(sections):
            if section in ['EXTERNAL', 'INTERNAL']:
                current_section = section.lower()
            elif current_section:
                # Find all property names in this section
                props = re.findall(r'^\s*(\w+)\s*(?:%.*)?$', section, re.MULTILINE)
                properties[current_section].extend(props)

    # Parse methods
    method_blocks = re.findall(r'methods(.*?)end', content, re.DOTALL)
    for block in method_blocks:
        methods.extend(re.findall(r'function\s+(?:.*?=\s*)?(\w+)', block))

    return class_name, properties, [m for m in methods if m != 'end']

def generate_plantuml(classes):
    puml = ["@startuml", 
            "hide empty members",
            "skinparam classAttributeIconSize 0",
            "skinparam classFontSize 12",
            "skinparam classFontName Courier"]
    
    for cls in classes:
        puml.append(f"class {cls['name']} {{")
        puml.append("  ' Externally Set Attributes")
        for prop in cls['props']['external']:
            if prop not in ['methods', 'end', 'properties']:
                puml.append(f"  + {prop}")
        
        puml.append("  ' Internally Computed Attributes")
        for prop in cls['props']['internal']:
            if prop not in ['methods', 'end', 'properties']:
                puml.append(f"  - {prop}")

        puml.append("  ' Public Methods")
        # Add constructor
        if cls['name'] in cls['methods']:
            puml.append(f"  + {cls['name']}()")
        
        puml.append("  ' Internal Methods")
        for method in cls['methods']:
            if method != cls['name'] and not method.startswith('end'):
                puml.append(f"  - {method}()")
        
        puml.append("}")
    
    puml.append("@enduml")
    return "\n".join(puml)

if __name__ == "__main__":
    matlab_files = glob.glob("True_*/*.m") + glob.glob("True_Sensors_Actuators/True_SC_Reaction_Wheel.m")
    
    parsed_classes = []  # Initialize the list
    
    for file in matlab_files:
        name, props, methods = parse_matlab_class(file)
        parsed_classes.append({
            'name': name,
            'props': props,
            'methods': methods
        })

    with open("class_diagram.puml", "w") as f:
        f.write(generate_plantuml(parsed_classes)) 