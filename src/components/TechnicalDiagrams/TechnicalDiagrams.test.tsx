import React from 'react';
import { render, screen } from '@testing-library/react';
import TechnicalDiagrams from './TechnicalDiagrams';

describe('TechnicalDiagrams', () => {
  const defaultProps = {
    title: 'Test Diagram',
    description: 'Test description',
    imageUrl: '/test-image.png',
    caption: 'Test caption',
    altText: 'Test alt text'
  };

  it('renders without crashing', () => {
    render(<TechnicalDiagrams {...defaultProps} />);
    expect(screen.getByText('Test Diagram')).toBeInTheDocument();
  });

  it('renders the description', () => {
    render(<TechnicalDiagrams {...defaultProps} />);
    expect(screen.getByText('Test description')).toBeInTheDocument();
  });

  it('renders the image with correct alt text', () => {
    render(<TechnicalDiagrams {...defaultProps} />);
    const img = screen.getByRole('img', { name: 'Test alt text' });
    expect(img).toBeInTheDocument();
    expect(img).toHaveAttribute('src', '/test-image.png');
  });

  it('renders the caption', () => {
    render(<TechnicalDiagrams {...defaultProps} />);
    expect(screen.getByText(/Test caption/)).toBeInTheDocument();
  });
});