import Header from '@/views/components/Header/Header';
import Dashboard from '@/views/components/Dashboard/Dashboard';
import styles from './page.module.css';

export default function Home() {
  return (
    <>
      <div className={styles.desktopHeader}>
        <Header />
      </div>
      <Dashboard />
    </>
  );
}
