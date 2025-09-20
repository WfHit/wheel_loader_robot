#!/usr/bin/env python3
"""
SVG Validation Script for Wheel Loader Architecture Diagram
Usage: python3 validate_svg.py [filename]
"""

import sys
import xml.etree.ElementTree as ET
import re
from pathlib import Path

def validate_xml_syntax(filename):
    """Check if the SVG file has valid XML syntax"""
    try:
        ET.parse(filename)
        print(f"✓ {filename} has valid XML syntax")
        return True
    except ET.ParseError as e:
        print(f"✗ XML Parse Error in {filename}: {e}")
        return False
    except FileNotFoundError:
        print(f"✗ File not found: {filename}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False

def check_common_svg_issues(filename):
    """Check for common SVG/XML issues"""
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()

        issues = []

        # Check for unescaped ampersands
        unescaped_amp = re.findall(r'[^&]&[^amp;#]', content)
        if unescaped_amp:
            issues.append("Found unescaped ampersands (&) - should be &amp;")

        # Check for unclosed tags
        open_tags = re.findall(r'<(\w+)[^>]*>', content)
        close_tags = re.findall(r'</(\w+)>', content)
        self_closing = re.findall(r'<(\w+)[^>]*/>', content)

        # Remove self-closing tags from open_tags count
        for tag in self_closing:
            if tag in open_tags:
                open_tags.remove(tag)

        unmatched_tags = set(open_tags) - set(close_tags)
        if unmatched_tags:
            issues.append(f"Potentially unmatched tags: {unmatched_tags}")

        # Check SVG dimensions
        viewbox_match = re.search(r'viewBox="([^"]*)"', content)
        width_match = re.search(r'width="([^"]*)"', content)
        height_match = re.search(r'height="([^"]*)"', content)

        if viewbox_match and width_match and height_match:
            viewbox = viewbox_match.group(1).split()
            if len(viewbox) == 4:
                vb_width, vb_height = float(viewbox[2]), float(viewbox[3])
                svg_width = float(width_match.group(1))
                svg_height = float(height_match.group(1))

                if svg_width != vb_width or svg_height != vb_height:
                    issues.append(f"SVG dimensions ({svg_width}x{svg_height}) don't match viewBox ({vb_width}x{vb_height})")

        # Check for elements extending beyond viewBox
        if viewbox_match:
            viewbox = viewbox_match.group(1).split()
            if len(viewbox) == 4:
                max_x, max_y = float(viewbox[2]), float(viewbox[3])

                # Find all x, y coordinates
                coords = re.findall(r'[xy]\d*="(\d+(?:\.\d+)?)"', content)
                coords.extend(re.findall(r'[xy]\d*="(\d+(?:\.\d+)?)"', content))

                for coord in coords:
                    if float(coord) > max_x or float(coord) > max_y:
                        issues.append(f"Element coordinate {coord} exceeds viewBox bounds")
                        break

        if issues:
            print(f"⚠ Potential issues found in {filename}:")
            for issue in issues:
                print(f"  - {issue}")
            return False
        else:
            print(f"✓ {filename} passed additional checks")
            return True

    except Exception as e:
        print(f"✗ Error checking {filename}: {e}")
        return False

def main():
    """Main validation function"""
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "wheel_loader_logic_architecture.svg"

    if not Path(filename).exists():
        print(f"✗ File not found: {filename}")
        sys.exit(1)

    print(f"Validating SVG file: {filename}")
    print("=" * 50)

    xml_valid = validate_xml_syntax(filename)
    issues_check = check_common_svg_issues(filename)

    print("=" * 50)
    if xml_valid and issues_check:
        print("✓ All checks passed! SVG file is valid.")
        sys.exit(0)
    else:
        print("✗ Validation failed. Please fix the issues above.")
        sys.exit(1)

if __name__ == "__main__":
    main()
