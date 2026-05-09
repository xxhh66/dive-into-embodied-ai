import React, {type ReactNode} from 'react';
import {ThemeClassNames} from '@docusaurus/theme-common';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import {useLocation} from '@docusaurus/router';
import Link from '@docusaurus/Link';
import TOC from '@theme/TOC';
import {Gamepad2} from 'lucide-react';
// Single source of truth for playground pages (shared with PlaygroundHeader).
import {PLAYGROUNDS} from '@site/src/components/PlaygroundHeader';

function normalizePath(pathname: string): string {
  return pathname.replace(/\/$/, '');
}

function getInteractiveHref(pathname: string): string | undefined {
  const currentPath = normalizePath(pathname);
  return PLAYGROUNDS.find((playground) => {
    const readingPath = normalizePath(playground.readingHref);
    return currentPath === readingPath || currentPath.endsWith(readingPath);
  })?.playgroundHref;
}

function InteractiveModeButton({href}: {href: string}) {
  return (
    <Link
      to={href}
      aria-label="打开交互模式"
      style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        gap: 6,
        padding: '10px 14px',
        marginBottom: 14,
        borderRadius: 10,
        background: 'var(--ifm-color-primary)',
        color: '#ffffff',
        fontWeight: 600,
        fontSize: 13,
        letterSpacing: '0.02em',
        textDecoration: 'none',
        boxShadow: '0 4px 12px -4px rgba(15, 23, 42, 0.35)',
      }}>
      <Gamepad2 size={16} aria-hidden="true" />
      <span>交互模式</span>
    </Link>
  );
}

export default function DocItemTOCDesktop(): ReactNode {
  const {toc, frontMatter} = useDoc();
  const {pathname} = useLocation();
  const interactiveHref = getInteractiveHref(pathname);

  return (
    <div
      style={{
        position: 'sticky',
        top: 'calc(var(--ifm-navbar-height) + 1rem)',
        maxHeight: 'calc(100vh - (var(--ifm-navbar-height) + 2rem))',
        overflowY: 'auto',
      }}>
      {interactiveHref && <InteractiveModeButton href={interactiveHref} />}
      <TOC
        toc={toc}
        minHeadingLevel={frontMatter.toc_min_heading_level}
        maxHeadingLevel={frontMatter.toc_max_heading_level}
        className={ThemeClassNames.docs.docTocDesktop}
      />
    </div>
  );
}
