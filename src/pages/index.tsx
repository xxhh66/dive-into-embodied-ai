import React, { useEffect, useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

const TYPEWRITER_PHRASES = [
  '从认知地图到可跑项目',
  '从感知到决策到控制',
  '从仿真到真机部署',
  '从论文到工程实践',
];

function Typewriter() {
  const [text, setText] = useState('');
  useEffect(() => {
    let phraseIndex = 0;
    let charIndex = 0;
    let deleting = false;
    let timer: ReturnType<typeof setTimeout>;

    const tick = () => {
      const word = TYPEWRITER_PHRASES[phraseIndex];
      charIndex += deleting ? -1 : 1;
      setText(word.substring(0, charIndex));

      let delay = deleting ? 45 : 90;
      if (!deleting && charIndex === word.length) {
        delay = 2200;
        deleting = true;
      } else if (deleting && charIndex === 0) {
        deleting = false;
        phraseIndex = (phraseIndex + 1) % TYPEWRITER_PHRASES.length;
        delay = 400;
      }
      timer = setTimeout(tick, delay);
    };

    timer = setTimeout(tick, 800);
    return () => clearTimeout(timer);
  }, []);

  return <span className="typewriter">{text}</span>;
}

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className="hero">
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <p className="hero__typewriter">
          <Typewriter />
        </p>
        <p style={{ fontSize: '1rem', color: 'var(--ifm-color-emphasis-600)' }}>
          给想入门、转行或找相关工作的同学准备
          <br />
          先补基础，再做项目，最后整理到简历和面试
        </p>
        <div style={{ marginTop: '1.5rem' }}>
          <a
            className="button button--primary button--lg"
            href="/dive-into-embodied-ai/docs/overview/intro"
          >
            开始学习
          </a>
          <a
            className="button button--secondary button--lg"
            href="https://github.com/datawhalechina/dive-into-embodied-ai"
            style={{ marginLeft: '1rem' }}
          >
            GitHub
          </a>
        </div>
      </div>
    </header>
  );
}

export default function Home(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout title={siteConfig.title} description={siteConfig.tagline}>
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
