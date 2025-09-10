#!/usr/bin/env python3
"""
Markdown to PDF Converter Tool

This script converts markdown files to PDF format with professional styling.
Designed for converting technical documentation, design documents, and README files.

Usage:
    python3 markdown_to_pdf.py <input_file.md> [output_file.pdf]
    python3 markdown_to_pdf.py --help

Examples:
    python3 markdown_to_pdf.py ../design/wheel_loader_robot_design.md
    python3 markdown_to_pdf.py ../README.md README.pdf
    python3 markdown_to_pdf.py ../docs/technical_spec.md ../docs/technical_spec.pdf

Author: PX4 Development Team
License: BSD 3-Clause
"""

import argparse
import markdown
import weasyprint
import sys
import os
import tempfile
from pathlib import Path
import re
import base64
import mimetypes

def get_document_title(markdown_content):
    """Extract the first heading as document title"""
    lines = markdown_content.split('\n')
    for line in lines:
        if line.startswith('# '):
            return line[2:].strip()
    return "Document"

def process_images(html_content, markdown_file_path):
    """Process images in HTML content to embed them as base64 data URIs"""
    base_dir = Path(markdown_file_path).parent

    # Find all img tags
    img_pattern = r'<img([^>]*?)src=["\']([^"\']*?)["\']([^>]*?)>'

    def replace_img(match):
        pre_src = match.group(1)
        src = match.group(2)
        post_src = match.group(3)

        # Skip if already a data URI
        if src.startswith('data:'):
            return match.group(0)

        # Skip if absolute URL
        if src.startswith(('http://', 'https://')):
            return match.group(0)

        # Resolve relative path
        img_path = base_dir / src
        if not img_path.exists():
            print(f"Warning: Image not found: {img_path}")
            return match.group(0)

        try:
            # Read image and convert to base64
            with open(img_path, 'rb') as f:
                img_data = f.read()

            # Get MIME type
            mime_type, _ = mimetypes.guess_type(str(img_path))
            if not mime_type:
                mime_type = 'image/png'  # default

            # Create data URI
            b64_data = base64.b64encode(img_data).decode('utf-8')
            data_uri = f"data:{mime_type};base64,{b64_data}"

            return f'<img{pre_src}src="{data_uri}"{post_src}>'

        except Exception as e:
            print(f"Warning: Could not process image {img_path}: {e}")
            return match.group(0)

    return re.sub(img_pattern, replace_img, html_content)

def process_code_blocks(html_content):
    """Process code blocks to add landscape orientation for better readability"""
    # Find all pre tags with code
    pre_pattern = r'<pre([^>]*)>(.*?)</pre>'

    def replace_pre(match):
        pre_attrs = match.group(1)
        code_content = match.group(2)

        # Count lines and check if code block is long enough for landscape
        lines = code_content.count('\n') + 1
        line_length = max(len(line.strip()) for line in code_content.split('\n') if line.strip())

        # Use landscape for code blocks with many lines or very long lines
        if lines > 10 or line_length > 80:
            # Wrap in a div with landscape class
            return f'<div class="code-landscape"><pre{pre_attrs} class="landscape">{code_content}</pre></div>'
        else:
            return f'<pre{pre_attrs}>{code_content}</pre>'

    return re.sub(pre_pattern, replace_pre, html_content, flags=re.DOTALL)

def markdown_to_pdf(markdown_file, output_pdf):
    """Convert a markdown file to PDF using markdown and weasyprint"""

    try:
        # Read the markdown file
        with open(markdown_file, 'r', encoding='utf-8') as f:
            markdown_content = f.read()

        # Extract document title for the PDF
        doc_title = get_document_title(markdown_content)

        # Convert markdown to HTML
        md = markdown.Markdown(extensions=['tables', 'toc', 'codehilite', 'fenced_code'])
        html_content = md.convert(markdown_content)

        # Process images to embed them as base64
        html_content = process_images(html_content, markdown_file)

        # Process code blocks for landscape orientation
        html_content = process_code_blocks(html_content)

        # Create a complete HTML document with professional CSS styling
        html_document = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <title>{doc_title}</title>
            <style>
                body {{
                    font-family: 'Arial', 'Helvetica', sans-serif;
                    line-height: 1.5;
                    color: #333;
                    max-width: none;
                    margin: 0;
                    padding: 0;
                    background-color: white;
                    font-size: 11pt;
                }}
                h1 {{
                    color: #2c3e50;
                    border-bottom: 3px solid #3498db;
                    padding-bottom: 10px;
                    font-size: 2.2em;
                    margin-top: 30px;
                    margin-bottom: 20px;
                    page-break-inside: avoid;
                }}
                h2 {{
                    color: #2c3e50;
                    border-bottom: 2px solid #3498db;
                    padding-bottom: 8px;
                    font-size: 1.8em;
                    margin-top: 25px;
                    margin-bottom: 15px;
                    page-break-inside: avoid;
                }}
                h3 {{
                    color: #34495e;
                    font-size: 1.4em;
                    margin-top: 20px;
                    margin-bottom: 10px;
                    page-break-inside: avoid;
                }}
                h4 {{
                    color: #34495e;
                    font-size: 1.2em;
                    margin-top: 15px;
                    margin-bottom: 8px;
                    page-break-inside: avoid;
                }}
                h5, h6 {{
                    color: #34495e;
                    font-size: 1.1em;
                    margin-top: 12px;
                    margin-bottom: 6px;
                }}
                p {{
                    margin-bottom: 12px;
                    text-align: justify;
                    orphans: 2;
                    widows: 2;
                }}
                ul, ol {{
                    margin-bottom: 15px;
                    padding-left: 25px;
                }}
                li {{
                    margin-bottom: 5px;
                }}
                strong {{
                    color: #2c3e50;
                    font-weight: bold;
                }}
                em {{
                    font-style: italic;
                    color: #555;
                }}
                code {{
                    background-color: #f8f9fa;
                    padding: 2px 4px;
                    border-radius: 3px;
                    font-family: 'Courier New', 'Monaco', monospace;
                    font-size: 0.9em;
                    color: #e74c3c;
                }}
                pre {{
                    background-color: #f8f9fa;
                    padding: 15px;
                    border-radius: 5px;
                    border-left: 4px solid #3498db;
                    overflow-x: auto;
                    margin-bottom: 15px;
                    page-break-inside: avoid;
                    page-break-before: auto;
                    font-size: 9pt;
                    white-space: pre-wrap;
                    word-wrap: break-word;
                }}
                pre.landscape {{
                    page-break-before: always;
                    page-break-after: always;
                    margin: 0;
                    padding: 20px;
                    width: 100%;
                    box-sizing: border-box;
                }}
                @page code-landscape {{
                    size: A4 landscape;
                    margin: 1.5cm;
                }}
                .code-landscape {{
                    page: code-landscape;
                }}
                pre code {{
                    background-color: transparent;
                    padding: 0;
                    color: #333;
                    font-size: inherit;
                }}
                blockquote {{
                    border-left: 4px solid #3498db;
                    padding-left: 15px;
                    margin: 15px 0;
                    font-style: italic;
                    color: #555;
                    background-color: #f8f9fa;
                    padding: 10px 15px;
                    border-radius: 0 5px 5px 0;
                }}
                table {{
                    border-collapse: collapse;
                    width: 100%;
                    margin-bottom: 15px;
                    page-break-inside: avoid;
                }}
                th, td {{
                    border: 1px solid #ddd;
                    padding: 8px;
                    text-align: left;
                }}
                th {{
                    background-color: #f8f9fa;
                    font-weight: bold;
                    color: #2c3e50;
                }}
                tr:nth-child(even) {{
                    background-color: #f9f9f9;
                }}
                .toc {{
                    background-color: #f8f9fa;
                    padding: 20px;
                    border-radius: 5px;
                    margin-bottom: 30px;
                    border: 1px solid #e0e0e0;
                }}
                .toc h2 {{
                    margin-top: 0;
                    color: #2c3e50;
                }}
                hr {{
                    border: none;
                    height: 2px;
                    background-color: #e0e0e0;
                    margin: 30px 0;
                }}
                @page {{
                    margin: 2cm 1.5cm;
                    size: A4 portrait;
                    @top-center {{
                        content: "{doc_title}";
                        font-size: 9pt;
                        color: #666;
                        border-bottom: 1px solid #ddd;
                        padding-bottom: 3px;
                        margin-bottom: 1cm;
                    }}
                    @bottom-center {{
                        content: "Page " counter(page) " of " counter(pages);
                        font-size: 9pt;
                        color: #666;
                        border-top: 1px solid #ddd;
                        padding-top: 3px;
                        margin-top: 1cm;
                    }}
                }}
                .page-break {{
                    page-break-before: always;
                }}
                .no-break {{
                    page-break-inside: avoid;
                }}
                img {{
                    max-width: 100%;
                    height: auto;
                    display: block;
                    margin: 10px auto;
                    border-radius: 5px;
                    box-shadow: 0 2px 5px rgba(0,0,0,0.1);
                }}
                figure {{
                    margin: 20px 0;
                    text-align: center;
                    page-break-inside: avoid;
                }}
                figcaption {{
                    font-style: italic;
                    color: #666;
                    margin-top: 5px;
                    font-size: 0.9em;
                }}
            </style>
        </head>
        <body>
            {html_content}
        </body>
        </html>
        """

        # Convert HTML to PDF using WeasyPrint
        try:
            # Create HTML document object
            html_doc = weasyprint.HTML(string=html_document, base_url=str(Path(markdown_file).parent))

            # Write PDF directly
            html_doc.write_pdf(output_pdf)
            print(f"✓ PDF created successfully: {output_pdf}")

        except Exception as e:
            print(f"Error creating PDF: {e}")
            print("Attempting alternative PDF generation method...")

            try:
                # Alternative method: create temporary HTML file
                with tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False, encoding='utf-8') as temp_html:
                    temp_html.write(html_document)
                    temp_html_path = temp_html.name

                # Create PDF from temporary HTML file
                html_doc = weasyprint.HTML(filename=temp_html_path)
                html_doc.write_pdf(output_pdf)

                # Clean up temporary file
                os.unlink(temp_html_path)

                print(f"✓ PDF created successfully: {output_pdf}")

            except Exception as e2:
                print(f"PDF creation failed: {e2}")
                return False

        return True

    except FileNotFoundError:
        print(f"Error: Input file '{markdown_file}' not found")
        return False
    except Exception as e:
        print(f"Error converting to PDF: {e}")
        return False

def main():
    """Main function with argument parsing"""
    parser = argparse.ArgumentParser(
        description="Convert Markdown files to PDF with professional styling",
        epilog="""
Examples:
  %(prog)s document.md                    # Creates document.pdf
  %(prog)s document.md output.pdf         # Creates output.pdf
  %(prog)s ../README.md ../README.pdf     # Convert with custom path
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        'input_file',
        help='Input markdown file path'
    )

    parser.add_argument(
        'output_file',
        nargs='?',
        help='Output PDF file path (optional, defaults to input filename with .pdf extension)'
    )

    parser.add_argument(
        '--version',
        action='version',
        version='Markdown to PDF Converter 1.0'
    )

    args = parser.parse_args()

    # Validate input file
    input_path = Path(args.input_file)
    if not input_path.exists():
        print(f"Error: Input file '{args.input_file}' does not exist")
        sys.exit(1)

    if not input_path.suffix.lower() in ['.md', '.markdown']:
        print(f"Warning: Input file '{args.input_file}' does not have a markdown extension")

    # Determine output file
    if args.output_file:
        output_path = Path(args.output_file)
    else:
        output_path = input_path.with_suffix('.pdf')

    # Ensure output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Convert the file
    print(f"Converting '{input_path}' to '{output_path}'...")

    if markdown_to_pdf(str(input_path), str(output_path)):
        if output_path.exists():
            print(f"✓ Conversion completed successfully!")
            print(f"  Output file: {output_path}")
            print(f"  File size: {output_path.stat().st_size / 1024:.1f} KB")
        else:
            print("✗ No output file created")
            sys.exit(1)
    else:
        print("✗ Conversion failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
