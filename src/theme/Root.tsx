/**
 * Root Component for Docusaurus
 * Wraps the entire app and injects the ChatBot component
 */

import React from 'react';
import ChatBot from '@site/src/components/ChatBot/ChatBot';

export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}
