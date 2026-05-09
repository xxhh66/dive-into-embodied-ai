import React from 'react';
import useBrokenLinks from '@docusaurus/useBrokenLinks';

type SvgComponent = React.ComponentType<React.SVGProps<SVGSVGElement>>;

interface FigureProps {
  src: string | SvgComponent;
  caption: string;
  width?: number | string;
  alt?: string;
  id?: string;
}

export default function Figure({src, caption, width = 560, alt, id}: FigureProps) {
  useBrokenLinks().collectAnchor(id);

  const imageStyle =
    typeof width === 'number'
      ? {width: '100%', maxWidth: `${width}px`}
      : {width: '100%', maxWidth: width};

  const label = alt ?? caption;
  const isSvgComponent = typeof src === 'function';

  return (
    <figure className="doc-figure" id={id}>
      {isSvgComponent ? (
        React.createElement(src, {
          role: 'img',
          'aria-label': label,
          style: imageStyle,
        })
      ) : (
        <img src={src} alt={label} style={imageStyle} />
      )}
      <figcaption>{caption}</figcaption>
    </figure>
  );
}
