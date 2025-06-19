#!/usr/bin/env python3
"""
Math MCP Server for Dino AI

Provides mathematical operations and calculations via MCP protocol.
"""

from fastmcp import FastMCP

mcp = FastMCP("MathServer")

@mcp.tool()
def add(a: float, b: float) -> float:
    """
    Add two numbers and return their sum.
    
    Args:
        a: First number
        b: Second number
        
    Returns:
        Sum of a and b
    """
    return a + b

@mcp.tool()
def multiply(a: float, b: float) -> float:
    """
    Multiply two numbers and return their product.
    
    Args:
        a: First number
        b: Second number
        
    Returns:
        Product of a and b
    """
    return a * b

@mcp.tool()
def subtract(a: float, b: float) -> float:
    """
    Subtract b from a and return the result.
    
    Args:
        a: First number (minuend)
        b: Second number (subtrahend)
        
    Returns:
        Difference of a and b
    """
    return a - b

@mcp.tool()
def divide(a: float, b: float) -> float:
    """
    Divide a by b and return the result.
    
    Args:
        a: Dividend
        b: Divisor
        
    Returns:
        Quotient of a and b
        
    Raises:
        ValueError: If b is zero
    """
    if b == 0:
        raise ValueError("Cannot divide by zero")
    return a / b

@mcp.tool()
def power(base: float, exponent: float) -> float:
    """
    Raise base to the power of exponent.
    
    Args:
        base: Base number
        exponent: Exponent
        
    Returns:
        base raised to the power of exponent
    """
    return base ** exponent

@mcp.tool()
def square_root(x: float) -> float:
    """
    Calculate the square root of x.
    
    Args:
        x: Number to find square root of
        
    Returns:
        Square root of x
        
    Raises:
        ValueError: If x is negative
    """
    if x < 0:
        raise ValueError("Cannot take square root of negative number")
    return x ** 0.5

@mcp.tool()
def factorial(n: int) -> int:
    """
    Calculate the factorial of n.
    
    Args:
        n: Non-negative integer
        
    Returns:
        Factorial of n
        
    Raises:
        ValueError: If n is negative
    """
    if n < 0:
        raise ValueError("Factorial is not defined for negative numbers")
    if n == 0 or n == 1:
        return 1
    
    result = 1
    for i in range(2, n + 1):
        result *= i
    return result

@mcp.tool()
def percentage(part: float, whole: float) -> float:
    """
    Calculate what percentage part is of whole.
    
    Args:
        part: Part value
        whole: Whole value
        
    Returns:
        Percentage (0-100)
        
    Raises:
        ValueError: If whole is zero
    """
    if whole == 0:
        raise ValueError("Cannot calculate percentage with zero whole")
    return (part / whole) * 100

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Math MCP Server")
    parser.add_argument("--host", default="127.0.0.1", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    
    args = parser.parse_args()
    
    print(f"Starting Math MCP Server on {args.host}:{args.port}")
    mcp.run(transport="sse", host=args.host, port=args.port)