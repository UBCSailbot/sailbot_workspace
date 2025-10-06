// this is a very gimmicky way of setting theme without using cookies
// i dont think its worth it so we should just use cookies instead lol

export const SET_THEME = `
(function(){
  try {
    var stored = localStorage.getItem('theme');
    document.documentElement.classList.toggle(stored ?? 'dark');
  } catch (e) {}
})();
`;
