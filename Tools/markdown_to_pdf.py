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

def get_document_title(markdown_content):
    """Extract the first heading as document title"""
    lines = markdown_content.split('\n')
    for line in lines:
        if line.startswith('# '):
            return line[2:].strip()
    return "Document"

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
                    line-height: 1.6;
                    color: #333;
                    max-width: 800px;
                    margin: 0 auto;
                    padding: 20px;
                    background-color: white;
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
                }}
                pre code {{
                    background-color: transparent;
                    padding: 0;
                    color: #333;
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
                    margin: 2.5cm;
                    size: A4;
                    @top-center {{
                        content: "{doc_title}";
                        font-size: 10pt;
                        color: #666;
                        border-bottom: 1px solid #ddd;
                        padding-bottom: 5px;
                    }}
                    @bottom-center {{
                        content: "Page " counter(page) " of " counter(pages);
                        font-size: 10pt;
                        color: #666;
                        border-top: 1px solid #ddd;
                        padding-top: 5px;
                    }}
                }}
                .page-break {{
                    page-break-before: always;
                }}
                .no-break {{
                    page-break-inside: avoid;
                }}
            </style>
        </head>
        <body>
            {html_content}
        </body>
        </html>
        """

        # Convert HTML to PDF using WeasyPrint
        # Note: Due to compatibility issues with pydyf library, we'll create HTML output as primary method
        html_output = output_pdf.replace('.pdf', '.html')
        with open(html_output, 'w', encoding='utf-8') as f:
            f.write(html_document)

        print(f"HTML version created: {html_output}")
        print("Note: PDF conversion has compatibility issues. HTML file created instead.")
        print("You can open the HTML file in a browser and print/save as PDF manually.")

        # Try WeasyPrint if user specifically wants PDF
        try:
            html_doc = weasyprint.HTML(string=html_document)
            html_doc.write_pdf(output_pdf)
            print(f"PDF also created successfully: {output_pdf}")
        except Exception as we_error:
            print(f"PDF creation failed (expected due to library compatibility): {we_error}")
            # Copy HTML to PDF name for consistency
            import shutil
            shutil.copy2(html_output, output_pdf.replace('.pdf', '_converted.html'))

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
            print(f"✓ PDF created successfully: {output_path}")
            print(f"  File size: {output_path.stat().st_size / 1024:.1f} KB")
        else:
            # Check for HTML file instead
            html_path = output_path.with_suffix('.html')
            if html_path.exists():
                print(f"✓ HTML file created successfully: {html_path}")
                print(f"  File size: {html_path.stat().st_size / 1024:.1f} KB")
                print("  (PDF conversion had compatibility issues - HTML file created instead)")
            else:
                print("✗ No output file created")
                sys.exit(1)
    else:
        print("✗ Conversion failed")
        sys.exit(1)

if __name__ == "__main__":
    main()
