'use client';

import Image from 'next/image';
import { useState, useEffect, useRef } from 'react';
import gsap from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';
import Description from '@/public/AboutDescription';
import styles from './about.module.css';

gsap.registerPlugin(ScrollTrigger);

const About = () => {
  const [activeIndex, setActiveIndex] = useState(0);
  const [displayIndex, setDisplayIndex] = useState(0);

  const rootRef = useRef<HTMLDivElement>(null);
  const imageRefs = useRef<Array<HTMLDivElement | null>>([]);
  const numberRef = useRef<HTMLDivElement>(null);
  const dotRefs = useRef<(HTMLDivElement | null)[]>([]);
  const headingRef = useRef<HTMLDivElement>(null);
  const textRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!rootRef.current) return;

    const context = gsap.context(() => {
      // fade in all elements on page load
      gsap.from(
        `.${styles.imageColumn}, .${styles.progress}, .${styles.textColumn}`,
        {
          opacity: 0,
          y: 30,
          duration: 0.6,
          ease: 'power2.out',
          stagger: 0.15,
          delay: 0.15,
        },
      );

      // fade in the dataset link when it reaches the viewport
      gsap.from(`.${styles.datasetLink}`, {
        opacity: 0,
        y: 30,
        duration: 0.6,
        ease: 'power2.out',
        delay: 0.15,
        scrollTrigger: {
          trigger: `.${styles.datasetLink}`,
          start: 'top 80%',
        },
      });

      // make each image trigger a scroll animation
      imageRefs.current.forEach((image, i) => {
        if (!image) return;

        ScrollTrigger.create({
          trigger: image,
          start: 'top 70%',
          end: 'bottom 30%',
          onEnter: () => {
            setActiveIndex(i);

            // dot animation when scrolling down
            if (i >= 1 && i <= 3) {
              const dot = dotRefs.current[i - 1];
              dot && gsap.to(dot, {
                opacity: 0,
                y: -440,
                duration: 0.7,
                ease: 'power2.in',
              });
            }
          },
          onEnterBack: () => {
            setActiveIndex(i);

            // dot animation when scrolling up
            if (i >= 0 && i <= 2) {
              const dot = dotRefs.current[i];
              dot && gsap.to(dot, {
                opacity: 1,
                y: 0,
                duration: 0.35,
                ease: 'power2.in',
              });
            }
          },
        });
      });
    }, rootRef);

    return () => context.revert();
  }, []);

  useEffect(() => {
    if (!numberRef.current || !headingRef.current || !textRef.current) return;

    const fadeTargets = [
      numberRef.current,
      headingRef.current,
      textRef.current,
    ];

    const timeline = gsap.timeline();
    timeline
      .to(fadeTargets, {
        opacity: 0,
        y: -10,
        duration: 0.2,
        ease: 'power1.out',
        stagger: 0.1,
      })
      .add(() => setDisplayIndex(activeIndex))
      .set(fadeTargets, { y: 10 })
      .to(fadeTargets, {
        opacity: 1,
        y: 0,
        duration: 0.25,
        ease: 'power1.in',
        stagger: 0.15,
      });

    return () => {
      timeline.kill();
    };
  }, [activeIndex]);

  return (
    <div ref={rootRef}>
      <div className={styles.container}>
        {/* image column */}
        <div className={styles.imageColumn}>
          {Description.map((section, index) => (
            <div
              key={index}
              ref={(image) => {
                imageRefs.current[index] = image;
              }}
            >
              <div className={styles.imageCard}>
                <Image
                  className={styles.image}
                  src={section.imageSrc}
                  width={460}
                  height={520}
                  alt={section.imageAlt}
                  priority
                />
              </div>
            </div>
          ))}
        </div>

        {/* progress bar */}
        <div className={styles.progress}>
          <div className={styles.progressFadeTop} />
          <p ref={numberRef} className={styles.progressNumber}>
            0{displayIndex + 1}
          </p>

          <div className={styles.progressLineWrap}>
            {[0, 1, 2].map((dotIdx) => (
              <div
                key={dotIdx}
                ref={(el) => {
                  dotRefs.current[dotIdx] = el;
                }}
                className={styles.progressDot}
                style={{ marginBottom: `${80 - 40 * dotIdx}px` }}
              />
            ))}
          </div>
        </div>

        {/* text section */}
        <div className={styles.textColumn}>
          <div className={styles.headingRow}>
            <h2 className={styles.aboutTitle}>ABOUT US</h2>
            <h3>/</h3>
            <h3 ref={headingRef} className={styles.sectionTitle}>
              {Description[displayIndex].title}
            </h3>
          </div>

          <p ref={textRef} className={styles.bodyText}>
            {Description[displayIndex].content}
          </p>
        </div>
      </div>

      {/* dataset link */}
      <div className={styles.datasetSection}>
        <a href='/download' className={styles.datasetAnchor}>
          <p className={styles.datasetLink}>View our datasets →</p>
        </a>
      </div>
    </div>
  );
};

export default About;
