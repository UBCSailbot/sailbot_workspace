import Header from '@/views/components/Header/Header';
import Download from '@/views/components/DatasetDownload/Download';
import MobileNav from '@/views/components/MobileNav/MobileNav';
import styles from '../page.module.css';

export default function Home() {
  return (
    <>
      <div className={styles.desktopHeader}>
        <Header />
      </div>
      <MobileNav />
      <Download />
    </>
  );
}
