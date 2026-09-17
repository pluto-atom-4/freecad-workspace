#!/usr/bin/env python3
"""
Pure Python VRML tokenizer / brace matcher.

Provides a single function: find_matching_brace() which safely finds matching braces
while ignoring braces inside strings (with escape handling) and comments.

No external dependencies — suitable for headless VRML/PROTO processing.

Usage:
    from vrml_lexer import find_matching_brace

    vrml_text = 'Shape { geometry Box { } }'
    close_pos = find_matching_brace(vrml_text, 6)  # Find closing } for { at pos 6
    print(vrml_text[close_pos])  # Prints: }
"""


def find_matching_brace(
    text: str,
    open_pos: int,
    open_char: str = "{",
    close_char: str = "}",
) -> int:
    """
    Find the position of the closing brace/bracket matching the opening brace at open_pos.

    Handles:
    - Strings: braces inside "..." are ignored (handles escaped quotes \")
    - Comments: braces inside # ... (to EOL) are ignored
    - Nested braces: correctly counts matching open/close balance

    Args:
        text: Input VRML/PROTO text
        open_pos: Position of the opening brace (e.g., index of '{')
        open_char: The opening character to match (default '{')
        close_char: The closing character to match (default '}')

    Returns:
        Position of the matching closing brace in text

    Raises:
        ValueError: If EOF reached before finding matching close, or if
                    a string or comment is unterminated
    """
    if open_pos >= len(text) or text[open_pos] != open_char:
        raise ValueError(
            f"Character at position {open_pos} is not '{open_char}'"
        )

    brace_count = 1
    pos = open_pos + 1
    in_string = False
    in_comment = False

    while pos < len(text) and brace_count > 0:
        char = text[pos]

        # Handle comment (# to end of line)
        if not in_string and char == "#":
            in_comment = True
            pos += 1
            continue

        # Exit comment at EOL
        if in_comment and char == "\n":
            in_comment = False
            pos += 1
            continue

        # Skip processing braces if in comment or string
        if in_comment:
            pos += 1
            continue

        # Handle string escaping
        if in_string:
            if char == "\\":
                # Escape sequence: skip both \ and next char
                pos += 2
                continue
            elif char == '"':
                # End of string
                in_string = False
                pos += 1
                continue
            else:
                pos += 1
                continue

        # Handle string start
        if char == '"':
            in_string = True
            pos += 1
            continue

        # Count braces (only when not in string or comment)
        if char == open_char:
            brace_count += 1
        elif char == close_char:
            brace_count -= 1

        pos += 1

    # Validate termination
    if in_string:
        raise ValueError(
            f"Unterminated string starting before position {pos}: {text[max(0, pos-50):pos+10]!r}"
        )

    if in_comment:
        # Comments can extend to EOF, which is fine
        pass

    if brace_count > 0:
        raise ValueError(
            f"Unmatched '{open_char}' starting at position {open_pos}: "
            f"reached EOF with brace_count={brace_count}"
        )

    # Return position of the closing brace (one position before current pos)
    return pos - 1
