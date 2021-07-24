import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

const features = [
  {
    title: <>My website</>,
    imageUrl: 'img/undraw_figure.svg',
    // secondParagraph: (
    //   <>
    //     First
    //   </>
    // ),
    description: (
      <>
        Docusaurus was designed from the ground up to be easily installed and
        used to get your website up and running quickly.
      </>
    ),
  },
  {
    title: <>Focus on What Matters</>,
    imageUrl: 'img/undraw_docusaurus_tree.svg',
    // secondParagraph: (
    //   <>
    //     Second
    //   </>
    // ),
    description: (
      <>
        Docusaurus lets you focus on your docs, and we&apos;ll do the chores. Go
        ahead and move your docs into the <code>docs</code> directory.
      </>
    ),
  },
  {
    title: <>Powered by React</>,
    imageUrl: 'img/undraw_docusaurus_react.svg',
    // secondParagraph: (
    //   <>
    //     Third
    //   </>
    // ),
    description: (
      <>
        Very nice your website layout by reusing React. Docusaurus can
        be extended while reusing the same header and footer.
      </>
    ),
  },
];

function Feature({ imageUrl, title, description, secondParagraph }) {
  const imgUrl = useBaseUrl(imageUrl);
  return (
    <div className={clsx('col col--4', styles.feature)}>
      {imgUrl && (
        <div className="text--center">
          <img className={styles.featureImage} src={imgUrl} alt={title} />
        </div>
      )}
      <h3>{title}</h3>
      <p>{description}</p>
      <p>{secondParagraph}</p>
    </div>
  );
}

function Home() {
  const context = useDocusaurusContext();
  const { siteConfig = {} } = context;
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div class="col">
          <div class="container">
            <div class="container card">
              <div class="card__header">
                <div class="row">
                  <div class="col col--3">
                    <img
                      class="avatar__photo"
                      src="img/avatar.jpg"
                    />
                  </div>
                  <div class="avatar__intro col col--9">
                    <div class={styles.card}>Matthew Woo</div>
                    <small class={styles.card}>
                      UAV <b> Software</b> & <b> Hardware</b> Engineer
                    </small>
                  </div>
                </div>
              </div>
              <div class="card__body">
                <small class={styles.card}>
                <b><i>Software Stacks & Programs</i></b> :
                <b> PX4-Autopilot</b>, <b> ROS</b>, <b> Gazebo</b>, <b> Unity3D</b>, <b> MATLAB</b>
                </small>
              </div>
            </div>
          </div>
        </div>
        <div className="container">
          <h1 className="hero__title">{siteConfig.title}</h1>
          <div>
            <span className=""><i><b> {siteConfig.tagline} : </b></i></span>
            <span class="badge badge--info">PX4-Autopilot</span>
            <span class="badge badge--secondary">Motion Planning</span>
            <span class="badge badge--success">Gazebo</span>
            <span class="badge badge--warning">Unity3D</span>
          </div>
          <br></br>
          <row className={styles.buttons}>
            <Link
              className={clsx(
                'button button--secondary button--lg',
                styles.getStarted,
              )}
              to={useBaseUrl('docs/')}>
              More Info
            </Link>
          </row>
        </div>
      </header>

      <main>
        {features && features.length > 0 && (
          <section className={styles.features}>
            <div className="container">
              <div className="row">
                {features.map((props, idx) => (
                  <Feature key={idx} {...props} />
                ))}
              </div>
            </div>
          </section>
        )}
      </main>
    </Layout>
  );
}

export default Home;
