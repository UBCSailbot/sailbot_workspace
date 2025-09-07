import VoyageDescription from '@/public/VoyageDescription';
import styles from './about.module.css';

const About = () => {
  return (
    <div className={styles.about}>
      <div className={styles.description}>{VoyageDescription}</div>
    </div>
  );
};

export default About;
