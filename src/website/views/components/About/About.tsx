import VoyageDescription from '@/public/VoyageDescription';
import { DatasetDownload } from '../DatasetDownload/DatasetDownload';
import styles from './about.module.css';

const About = () => {
  return (
    <div className={styles.aboutContainer}>
      <div className={styles.about}>
        <h2>ABOUT US</h2>
        <p>{VoyageDescription}</p>
        <h2>VIEW OUR DATASETS</h2>
        <DatasetDownload />
      </div>
    </div>
  );
};

export default About;
