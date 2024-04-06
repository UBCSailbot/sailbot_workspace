/**
 * Initializes session storage data by attempting to load previously saved data
 * from the session storage using the provided key. If no data is found, it returns
 * the provided initial state.
 * @param {string} key - The key under which the data is stored in session storage.
 * @param {any} initialState - The initial state to return if no data is found.
 * @returns {any} The persisted data if found, otherwise the initial state.
 */
export const initSessionStorageData = (key, initialState) => {
  const persistedData = loadSessionStorageData(key);
  if (persistedData) {
    return persistedData;
  }
  return initialState;
};

/**
 * Loads data from session storage using the provided key.
 * @param {string} key - The key under which the data is stored in session storage.
 * @returns {any} The data retrieved from session storage, or undefined if not found.
 */
export const loadSessionStorageData = (key) => {
  try {
    const serializedData = sessionStorage.getItem(key);
    if (serializedData === null) {
      return undefined;
    }
    return JSON.parse(serializedData);
  } catch (error) {
    return undefined;
  }
};

/**
 * Saves data to session storage using the provided key.
 * @param {string} key - The key under which the data will be stored in session storage.
 * @param {any} data - The data to be stored in session storage.
 */
export const saveSessionStorageData = (key, data) => {
  try {
    const serializedData = JSON.stringify(data);
    sessionStorage.setItem(key, serializedData);
  } catch (error) {
    // Ignore write errors.
  }
};
