import React, {type ReactNode} from 'react';
import useBrokenLinks from '@docusaurus/useBrokenLinks';

interface DocTableProps {
  id?: string;
  caption: string;
  children: ReactNode;
}

export default function DocTable({id, caption, children}: DocTableProps) {
  useBrokenLinks().collectAnchor(id);

  return (
    <figure className="doc-table" id={id}>
      {children}
      <figcaption>{caption}</figcaption>
    </figure>
  );
}
