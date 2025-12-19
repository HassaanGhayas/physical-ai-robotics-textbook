import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { BentoGrid, Card } from '@/design-system/components';
import { FadeIn } from '@/design-system/animations';
import type { BentoGridItem } from '@/design-system/components';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="py-16 px-4 text-center">
      <div className="container-max-width">
        <FadeIn duration={0.6} direction="up" distance={30}>
          <Heading as="h1" className="text-5xl md:text-6xl font-bold mb-6 text-grayscale-900 dark:text-grayscale-50">
            {siteConfig.title}
          </Heading>
          <p className="text-xl md:text-2xl mb-8 text-grayscale-600 dark:text-grayscale-400 max-w-3xl mx-auto">
            {siteConfig.tagline}
          </p>
          <div className="flex gap-4 justify-center flex-wrap">
            <Link
              className="button button--primary button--lg bg-grayscale-900 hover:bg-grayscale-800 dark:bg-grayscale-50 dark:hover:bg-grayscale-100 dark:text-grayscale-950 text-grayscale-0 border-0"
              to="/docs/book/introduction">
              Start Reading
            </Link>
            <Link
              className="button button--secondary button--lg bg-grayscale-200 hover:bg-grayscale-300 dark:bg-grayscale-800 dark:hover:bg-grayscale-700 text-grayscale-900 dark:text-grayscale-50 border-0"
              to="https://github.com/HassaanGhayas/physical-ai-robotics-textbook">
              View on GitHub
            </Link>
          </div>
        </FadeIn>
      </div>
    </header>
  );
}

function HomepageFeatures(): ReactNode {
  const features: BentoGridItem[] = [
    {
      id: 'hardware',
      content: (
        <FadeIn duration={0.5} delay={0.1} direction="up" distance={20}>
          <Card variant="elevated" padding="lg" className="h-full">
            <div className="mb-4">
              <div className="text-4xl mb-2">🤖</div>
              <Heading as="h3" className="text-xl font-semibold mb-2">Hardware Requirements</Heading>
              <p className="text-grayscale-600 dark:text-grayscale-400">
                Essential components and specifications for building physical AI systems
              </p>
            </div>
            <Link to="/docs/book/hardware-requirements" className="text-sm font-medium hover:underline">
              Learn more →
            </Link>
          </Card>
        </FadeIn>
      ),
      span: { mobile: 1, tablet: 1, desktop: 1 },
      aspectRatio: 'auto',
    },
    {
      id: 'nervous-system',
      content: (
        <FadeIn duration={0.5} delay={0.2} direction="up" distance={20}>
          <Card variant="elevated" padding="lg" className="h-full">
            <div className="mb-4">
              <div className="text-4xl mb-2">🧠</div>
              <Heading as="h3" className="text-xl font-semibold mb-2">Robotic Nervous System</Heading>
              <p className="text-grayscale-600 dark:text-grayscale-400">
                Communication protocols and sensor integration for embodied AI
              </p>
            </div>
            <Link to="/docs/book/robotic-nervous-system" className="text-sm font-medium hover:underline">
              Learn more →
            </Link>
          </Card>
        </FadeIn>
      ),
      span: { mobile: 1, tablet: 1, desktop: 1 },
      aspectRatio: 'auto',
    },
    {
      id: 'digital-twin',
      content: (
        <FadeIn duration={0.5} delay={0.3} direction="up" distance={20}>
          <Card variant="elevated" padding="lg" className="h-full">
            <div className="mb-4">
              <div className="text-4xl mb-2">💻</div>
              <Heading as="h3" className="text-xl font-semibold mb-2">Digital Twin</Heading>
              <p className="text-grayscale-600 dark:text-grayscale-400">
                Virtual simulation and testing environments for robot development
              </p>
            </div>
            <Link to="/docs/book/digital-twin" className="text-sm font-medium hover:underline">
              Learn more →
            </Link>
          </Card>
        </FadeIn>
      ),
      span: { mobile: 1, tablet: 1, desktop: 1 },
      aspectRatio: 'auto',
    },
    {
      id: 'ai-brain',
      content: (
        <FadeIn duration={0.5} delay={0.4} direction="up" distance={20}>
          <Card variant="elevated" padding="lg" className="h-full">
            <div className="mb-4">
              <div className="text-4xl mb-2">🎯</div>
              <Heading as="h3" className="text-xl font-semibold mb-2">AI Robot Brain</Heading>
              <p className="text-grayscale-600 dark:text-grayscale-400">
                Machine learning models and decision-making architectures
              </p>
            </div>
            <Link to="/docs/book/ai-robot-brain" className="text-sm font-medium hover:underline">
              Learn more →
            </Link>
          </Card>
        </FadeIn>
      ),
      span: { mobile: 1, tablet: 1, desktop: 1 },
      aspectRatio: 'auto',
    },
    {
      id: 'vla',
      content: (
        <FadeIn duration={0.5} delay={0.5} direction="up" distance={20}>
          <Card variant="elevated" padding="lg" className="h-full">
            <div className="mb-4">
              <div className="text-4xl mb-2">👁️</div>
              <Heading as="h3" className="text-xl font-semibold mb-2">Vision-Language-Action</Heading>
              <p className="text-grayscale-600 dark:text-grayscale-400">
                Multimodal AI models for perception and action generation
              </p>
            </div>
            <Link to="/docs/book/vision-language-action" className="text-sm font-medium hover:underline">
              Learn more →
            </Link>
          </Card>
        </FadeIn>
      ),
      span: { mobile: 1, tablet: 1, desktop: 1 },
      aspectRatio: 'auto',
    },
    {
      id: 'assessments',
      content: (
        <FadeIn duration={0.5} delay={0.6} direction="up" distance={20}>
          <Card variant="elevated" padding="lg" className="h-full">
            <div className="mb-4">
              <div className="text-4xl mb-2">📝</div>
              <Heading as="h3" className="text-xl font-semibold mb-2">Assessments</Heading>
              <p className="text-grayscale-600 dark:text-grayscale-400">
                Practical exercises and evaluation metrics for learning
              </p>
            </div>
            <Link to="/docs/book/assessments" className="text-sm font-medium hover:underline">
              Learn more →
            </Link>
          </Card>
        </FadeIn>
      ),
      span: { mobile: 1, tablet: 1, desktop: 1 },
      aspectRatio: 'auto',
    },
  ];

  return (
    <section className="py-16 px-4">
      <div className="container-max-width">
        <FadeIn duration={0.6} direction="up" distance={20}>
          <div className="text-center mb-12">
            <Heading as="h2" className="text-3xl md:text-4xl font-bold mb-4 text-grayscale-900 dark:text-grayscale-50">
              Explore the Book
            </Heading>
            <p className="text-lg text-grayscale-600 dark:text-grayscale-400 max-w-2xl mx-auto">
              A comprehensive journey through hardware, software, and AI for building intelligent robots
            </p>
          </div>
        </FadeIn>
        <BentoGrid
          items={features}
          columns={{ mobile: 1, tablet: 2, desktop: 3 }}
          gap={{ mobile: '16px', tablet: '20px', desktop: '24px' }}
        />
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description={siteConfig.tagline}>
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
