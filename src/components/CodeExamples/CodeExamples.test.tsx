import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import CodeExamples from './CodeExamples';

describe('CodeExamples', () => {
  const defaultProps = {
    title: 'Test Code Example',
    description: 'Test description',
    code: 'console.log("Hello, world!");',
    language: 'javascript'
  };

  it('renders without crashing', () => {
    render(<CodeExamples {...defaultProps} />);
    expect(screen.getByText('Test Code Example')).toBeInTheDocument();
  });

  it('renders the description', () => {
    render(<CodeExamples {...defaultProps} />);
    expect(screen.getByText('Test description')).toBeInTheDocument();
  });

  it('renders the code block', () => {
    render(<CodeExamples {...defaultProps} />);
    const codeBlock = screen.getByText(/console\.log/);
    expect(codeBlock).toBeInTheDocument();
  });

  it('renders copy button', () => {
    render(<CodeExamples {...defaultProps} />);
    const copyButton = screen.getByTitle('Copy to clipboard');
    expect(copyButton).toBeInTheDocument();
  });

  it('renders expand button when code is long', () => {
    const longCode = `function example() {
  console.log("line 1");
  console.log("line 2");
  console.log("line 3");
  console.log("line 4");
  console.log("line 5");
  console.log("line 6");
  console.log("line 7");
  console.log("line 8");
  console.log("line 9");
  console.log("line 10");
  console.log("line 11");
  console.log("line 12");
  console.log("line 13");
  console.log("line 14");
  console.log("line 15");
  console.log("line 16");
}`;
    render(<CodeExamples {...defaultProps} code={longCode} />);
    const expandButton = screen.getByText('Expand Code');
    expect(expandButton).toBeInTheDocument();
  });
});