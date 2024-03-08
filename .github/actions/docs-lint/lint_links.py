## IMPORTS
import os
import sys
import re
import json
import argparse

## CONSTANTS
REGEX_PATTERN = r"(?<!!)\[.*?\]\(\s*https?:\/\/[^\(\)]+\)(?!\{\s*:?\s*target\s*=\s*(?:\s*_blank\s*|\s*\"\s*_blank\s*\"\s*)\})"
LINK_REGEX = r"https?:\/\/(www\.)?[-a-zA-Z0-9@:%._\+~#=]{1,256}\.[a-zA-Z0-9()]{1,6}\b([-a-zA-Z0-9()@:%_\+.~#?&//=]*)"
PASSED_MSG = "[PASSED]"
FAILED_MSG = "[FAILED]"
ERROR_MSG1 = "External links should redirect to a new tab. Change the link to "
ERROR_MSG2 = "{target=_blank}"
ROOT_DEFAULT = "./"
CONFIG_DEFAULT = ""


# Annotation strings for GitHub error annotations
annotations = []


## MAIN LOGIC
def main():
    args = parseInputArguments()
    root = args.root if args.root else ROOT_DEFAULT
    config_file = args.config if args.config else CONFIG_DEFAULT

    # Perform the linting process
    ignore_patterns = get_ignore_patterns(config_file)
    ignore_files = get_ignore_files(config_file)
    markdown_files = get_markdown_files(root, ignore_files)
    passed = lint_markdown_files(markdown_files, REGEX_PATTERN, ignore_patterns)

    # If linting fails, print any annotations to stderr for GitHub and exit with status code 1
    if not passed:
        print("\n".join(annotations), file=sys.stderr)
        sys.exit(1)


## HELPER FUNCTIONS
def parseInputArguments():
    """
    Parses command line arguments for the root directory and configuration file.

    Returns:
        Namespace: A namespace object used for accessing command line arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--root', default=ROOT_DEFAULT, nargs='?', type=str, help='Path to root directory where linting begins')
    parser.add_argument('-c', '--config', default=CONFIG_DEFAULT, nargs='?', type=str, help='Path to a JSON configuration file specifying ignore files and patterns')
    args = parser.parse_args()
    return args


def get_ignore_patterns(config_file):
    """
    Obtain a list of patterns to ignore specified in the configuration file whose path is specified as a command line argument.

    Args:
        config_file (str): The path of the configuration file relative to the root.

    Returns:
        List[str]: A list of regex patterns to ignore when performing linting.
    """
    if not os.path.isfile(config_file):
        print("Warning: Configuration file not found", file=sys.stderr)
        return []

    ignore_patterns = []
    with open(config_file) as f:
        data = json.load(f)
        ignore_patterns = [row["pattern"] for row in data.get("ignorePatterns", []) if "pattern" in row]
    return ignore_patterns


def get_ignore_files(config_file):
    """
    Obtain a list of files to ignore specified in the configuration file whose path is specified as a command line argument.

    Args:
        config_file (str): The path of the configuration file relative to the root.

    Returns:
        List[str]: A list of markdown file paths (relative to the root) to ignore when performing linting.
    """
    if not os.path.isfile(config_file):
        print("Warning: Configuration file not found", file=sys.stderr)
        return []

    ignore_files = []
    with open(config_file) as f:
        data = json.load(f)
        ignore_files = [row["file"] for row in data.get("ignoreFiles", []) if "file" in row]
    return ignore_files


def get_markdown_files(root_dir, ignore_files):
    """
    Recursively searches for markdown files (.md) starting at a specified root directory.

    Args:
        root_dir (str): The root directory to start the search at.
        ignore_files (List[str]): A list of markdown file paths to ignore.

    Returns:
        List[str]: A list of markdown file paths relative to the root directory.
    """
    markdown_files = []
    markdown_matcher = re.compile(r".+\.md")
    for root, dirs, files in os.walk(root_dir):
        markdown_file_basenames = filter(lambda f: markdown_matcher.match(f) is not None, files)
        markdown_files_with_full_path = map(lambda f: os.path.join(root, f), markdown_file_basenames)
        markdown_files_to_keep = filter(lambda f: f not in ignore_files, markdown_files_with_full_path)
        markdown_files += list(markdown_files_to_keep)
    return markdown_files


def lint_markdown_files(files, pattern, ignore_patterns):
    """
    Lints all specified markdown files and checks for any links to outside the Sailbot Docs website
    that do not redirect to a new tab. If any such links exists, the linting process fails.

    Args:
        files (List[str]): A list of markdown file paths relative to some root directory.
        pattern (str): A raw string containing the regular expression pattern to be used for linting.
        ignore_patterns (List[str]): A list of regex patterns to ignore.

    Returns:
        bool: Returns True if the linting process succeeds for all markdown files and False otherwise.
    """
    passed = True
    num_passed = 0
    num_checks = len(files)

    for n, file in enumerate(files):
        check_passed, error_message = check_markdown_file(file, pattern, ignore_patterns)
        passed = (passed and check_passed)
        num_passed += int(check_passed)
        print_check_message(file, check_passed, n+1, error_message)

    print(f"{num_passed}/{num_checks} checks passed")

    return passed


def check_markdown_file(filename, pattern, ignore_patterns):
    """
    Lints a specified markdown file.

    Args:
        filename (str): The path to the markdown file relative to some root directory.
        pattern (str): A raw string containing the regular expression pattern to be used for linting.
        ignore_patterns (List[str]): A list of regex patterns to ignore.

    Returns:
        tuple[bool, str]: Returns a tuple containing two variables:
            1. A boolean variable that indicates if the check passes
            2. A string containing an error message. This string is empty if the check passes.
    """
    passed = True
    error_message_buffer = ""

    with open(filename) as file:
        for line_number, line_text in enumerate(file.readlines()):
            match = non_redirecting_hyperlinks(line_text, pattern, ignore_patterns)
            passed, buffer = prepare_error_messages(match, filename, line_number, passed)
            error_message_buffer += buffer
    return passed, error_message_buffer


def non_redirecting_hyperlinks(line_text, pattern, ignore_patterns):
    """
    Helper function that finds all hyperlinks missing a redirection attribute on a given line.

    Args:
        line_text (str): A line from a markdown file being linted.
        pattern (str): A raw string containing the regular expression pattern to be used for linting.
        ignore_patterns (List[str]): A list of regex patterns to ignore.

    Returns:
        List[str]: A list of hyperlinks missing a redirection attribute
    """
    match = re.findall(pattern, line_text, flags=re.M)
    ignore_links = []
    for ignore_pattern in ignore_patterns:
        expression = re.compile(ignore_pattern)
        ignore_links += list(filter(lambda l: expression.match(re.search(LINK_REGEX, l).group()[:-1]), match))
    match = list(filter(lambda l: l not in ignore_links, match))
    return match


def prepare_error_messages(match, filename, line_number, passed):
    """
    Helper function that generates error messages for hyperlinks without redirection attributes and modifies the passed bool if necessary.

    Args:
        match (list[str]): A list of strings representing the matched links.
        filename (str): The name of the file being checked for errors.
        line_number (int): The line number where the error occurred.
        passed (bool): A bool specifying if previous links have passed or not.

    Returns:
        tuple[bool, str]: Returns a tuple containing two variables:
            1. The previous passed bool if there are no failed hyperlinks and false if there are.
            2. A string containing the generated error message buffer.
    """
    error_message_buffer = ""

    if not match:
        return passed, error_message_buffer

    passed = False
    for link in match:
        error_message_buffer += f"\tLine {line_number+1}: {link}\n"
        annotations.append(f"::error file={filename},line={line_number+1}::{ERROR_MSG1 + link + ERROR_MSG2}")
    return passed, error_message_buffer


def print_check_message(filename, check_passed, check_number, error_message):
    """
    Prints the status of a markdown file after a check. Any errors will be printed along with
    the status.

    Args:
        filename (str): The path to the markdown file relative to some root directory.
        check_passed (bool): Whether the check on the markdown file passed or not.
        check_number (int): The check number.
        error_message (str): An error message (empty if the check passed).
    """
    status = PASSED_MSG if check_passed else FAILED_MSG
    print(f"Check {check_number}: {status} {filename}\n" + error_message)


if __name__ == '__main__':
    main()
