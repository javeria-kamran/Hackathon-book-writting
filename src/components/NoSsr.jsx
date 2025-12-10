import BrowserOnly from '@docusaurus/BrowserOnly';
import React from 'react';

function NoSsr({ children }) {
  return (
    <BrowserOnly>{() => <>{children}</>}</BrowserOnly>
  );
}

export default NoSsr;
