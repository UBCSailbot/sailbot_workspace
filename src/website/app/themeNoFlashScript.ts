// this is a very gimmicky way preventing screen flashes when
// switching themes. we run this before the page is rendered

export const SET_THEME = `
(function(){
  var stored = localStorage.getItem('theme');
  document.documentElement.classList.toggle(stored ?? 'dark');
})();
`;
