import Header from '@/views/components/Header/Header';
import About from '@/views/components/About/About';
import MobileNav from '@/views/components/MobileNav/MobileNav';
import styles from '../page.module.css';

export default function Home() {
  return (
    <>
      <div className={styles.desktopHeader}>
        <Header />
      </div>
      <MobileNav />
      <About />
    </>
  );
}
