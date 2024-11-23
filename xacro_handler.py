import xml.etree.ElementTree as ET
import tempfile
import os
import re

class XacroHandler:
    def __init__(self):
        self.temp_files = []  # Keep track of temporary files for cleanup
        self.properties = {}  # Store xacro properties/constants

    def __del__(self):
        # Cleanup temporary files
        for temp_file in self.temp_files:
            try:
                if os.path.exists(temp_file):
                    os.remove(temp_file)
            except Exception as e:
                print(f"Error cleaning up temp file {temp_file}: {e}")

    def process_xacro(self, xacro_path):
        """
        Process a XACRO file and return the path to the generated URDF
        """
        try:
            # Parse the XACRO file
            tree = ET.parse(xacro_path)
            root = tree.getroot()

            # Process the XACRO content
            processed_urdf = self.process_xacro_element(root)

            # Create a temporary file for the URDF output
            with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as tmp:
                urdf_path = tmp.name
                self.temp_files.append(urdf_path)

                # Write the processed URDF
                tree = ET.ElementTree(processed_urdf)
                tree.write(urdf_path, encoding='utf-8', xml_declaration=True)

            return urdf_path
        except Exception as e:
            print(f"Error processing XACRO file: {e}")
            return None

    def process_xacro_element(self, element):
        """
        Process a single XACRO element and its children
        """
        # Create a new element without the xacro namespace
        new_element = ET.Element(self.remove_namespace(element.tag))
        
        # Copy attributes
        for key, value in element.attrib.items():
            key = self.remove_namespace(key)
            if key not in ['xmlns:xacro']:  # Skip xacro namespace declarations
                new_element.set(key, value)

        # Process children
        for child in element:
            if 'property' in child.tag:
                # Handle xacro:property
                name = child.attrib.get('name')
                value = child.attrib.get('value')
                if name and value:
                    self.properties[name] = value
            else:
                # Process regular elements
                processed_child = self.process_xacro_element(child)
                if processed_child is not None:
                    new_element.append(processed_child)

        # Process text content
        if element.text and element.text.strip():
            new_element.text = self.resolve_properties(element.text)

        return new_element

    def remove_namespace(self, tag):
        """Remove namespace prefix from tag"""
        return tag.split('}')[-1] if '}' in tag else tag

    def resolve_properties(self, text):
        """Resolve ${property} references in text"""
        def replace_prop(match):
            prop_name = match.group(1)
            return self.properties.get(prop_name, match.group(0))

        pattern = r'\${([^}]+)}'
        return re.sub(pattern, replace_prop, text)