/**
 * Card Component Tests
 * Tests for Card component variants and states
 */

import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import { Card } from '@/design-system/components';

describe('Card', () => {
  it('renders card with children', () => {
    render(
      <Card testId="test-card">
        <p>Card content</p>
      </Card>
    );
    expect(screen.getByTestId('test-card')).toHaveTextContent('Card content');
  });

  it('applies default variant', () => {
    render(
      <Card testId="test-card" variant="default">
        Content
      </Card>
    );
    const card = screen.getByTestId('test-card');
    expect(card).toBeInTheDocument();
  });

  it('applies elevated variant', () => {
    render(
      <Card testId="test-card" variant="elevated">
        Content
      </Card>
    );
    const card = screen.getByTestId('test-card');
    expect(card).toBeInTheDocument();
  });

  it('applies outlined variant', () => {
    render(
      <Card testId="test-card" variant="outlined">
        Content
      </Card>
    );
    const card = screen.getByTestId('test-card');
    expect(card).toBeInTheDocument();
  });

  it('applies small padding', () => {
    render(
      <Card testId="test-card" padding="sm">
        Content
      </Card>
    );
    expect(screen.getByTestId('test-card')).toBeInTheDocument();
  });

  it('applies medium padding', () => {
    render(
      <Card testId="test-card" padding="md">
        Content
      </Card>
    );
    expect(screen.getByTestId('test-card')).toBeInTheDocument();
  });

  it('applies large padding', () => {
    render(
      <Card testId="test-card" padding="lg">
        Content
      </Card>
    );
    expect(screen.getByTestId('test-card')).toBeInTheDocument();
  });

  it('renders as interactive button when interactive prop is true', () => {
    render(
      <Card testId="test-card" interactive>
        Content
      </Card>
    );
    const card = screen.getByTestId('test-card');
    expect(card).toHaveAttribute('role', 'button');
    expect(card).toHaveAttribute('tabIndex', '0');
  });

  it('applies custom className', () => {
    render(
      <Card testId="test-card" className="custom-class">
        Content
      </Card>
    );
    const card = screen.getByTestId('test-card');
    expect(card).toHaveClass('custom-class');
  });
});
