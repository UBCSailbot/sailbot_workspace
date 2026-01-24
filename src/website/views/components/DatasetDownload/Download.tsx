import { DatasetDownload } from '../DatasetDownload/DatasetDownload';
import styles from './datasetDownload.module.css';

const Download = () => {
  return (
    <div className={styles.downloadContainer}>
      <div className={styles.downloadHeader}>
        <h2>DOWNLOAD OUR DATASETS</h2>
        {/* how do i make this link to actual time ;-; */}
        {/* <p>Last Updated: 0000-00-00 00:00:00</p>*/} 
      </div>
      <DatasetDownload />
    </div>
  );
}

export default Download;
