const LIGHT_MODE = 0;
const DARK_MODE = 1;
const LIGHT_MODE_TABLE_OF_CONTENTS_LINK_COLOR = "#000000"; // Black
const DARK_MODE_TABLE_OF_CONTENTS_LINK_COLOR = "#ffffff";  // White
const SAILBOT_BLUE = "#2f97ec";

// Sets the theme given a mode
function set_table_of_content_link_color(mode) {
    let linkColor = "";
    switch (mode) {
        case LIGHT_MODE:
            linkColor = LIGHT_MODE_TABLE_OF_CONTENTS_LINK_COLOR;
            break;
        case DARK_MODE:
            linkColor = DARK_MODE_TABLE_OF_CONTENTS_LINK_COLOR;
            break;
        default:
            console.log("Error determining website theme. Defaulting table of content link color to sailbot blue.");
            linkColor = SAILBOT_BLUE;
            break;
    }
    document.documentElement.style.setProperty("--md-table-of-contents-link-color", linkColor);
}

// Returns the new theme index given the current theme
function toggle_table_of_contents_link_color(current_mode) {
    switch (current_mode) {
        case LIGHT_MODE:
            return DARK_MODE;
        case DARK_MODE:
            return LIGHT_MODE;
        default:
            return -1;
    }
}

// An enslosure that keeps track of the mode and toggles the theme accordingly
const theme_enclosure = function(initial_mode) {
    let current_mode = initial_mode;
    return {
        setLinkColor: () => {set_table_of_content_link_color(current_mode);},
        toggleLinkColor: () => {current_mode = toggle_table_of_contents_link_color(current_mode); set_table_of_content_link_color(current_mode);}
    };
};

// Set the theme upon the window loading
var initial_mode = __md_get("__palette").index;
table_of_contents_theme = theme_enclosure(initial_mode);
table_of_contents_theme.setLinkColor();

// Set the theme when the light/dark mode button is clicked
const buttons = document.querySelectorAll("form.md-header__option > label.md-header__button");
buttons.forEach((button) => {
    button.addEventListener('click', table_of_contents_theme.toggleLinkColor);
});
