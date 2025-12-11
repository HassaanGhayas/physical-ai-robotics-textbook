import React from 'react';
import { render, screen } from '@testing-library/react';
import HardwareSpecs from './HardwareSpecs';

describe('HardwareSpecs', () => {
  const defaultProps = {
    name: 'Test Hardware',
    category: 'Test Category',
    specs: {
      cpu: 'Test CPU',
      gpu: 'Test GPU',
      memory: 'Test Memory',
      storage: 'Test Storage'
    },
    cost: {
      price: '$1000',
      reasoning: 'Test reasoning'
    },
    pros: ['Test pro 1', 'Test pro 2'],
    cons: ['Test con 1', 'Test con 2']
  };

  it('renders without crashing', () => {
    render(<HardwareSpecs {...defaultProps} />);
    expect(screen.getByText('Test Hardware')).toBeInTheDocument();
  });

  it('renders the category', () => {
    render(<HardwareSpecs {...defaultProps} />);
    expect(screen.getByText('Test Category')).toBeInTheDocument();
  });

  it('renders all spec items', () => {
    render(<HardwareSpecs {...defaultProps} />);
    expect(screen.getByText('Test CPU')).toBeInTheDocument();
    expect(screen.getByText('Test GPU')).toBeInTheDocument();
    expect(screen.getByText('Test Memory')).toBeInTheDocument();
    expect(screen.getByText('Test Storage')).toBeInTheDocument();
  });

  it('renders cost information', () => {
    render(<HardwareSpecs {...defaultProps} />);
    expect(screen.getByText('$1000')).toBeInTheDocument();
    expect(screen.getByText('Test reasoning')).toBeInTheDocument();
  });

  it('renders pros and cons', () => {
    render(<HardwareSpecs {...defaultProps} />);
    expect(screen.getByText('Test pro 1')).toBeInTheDocument();
    expect(screen.getByText('Test con 1')).toBeInTheDocument();
  });
});