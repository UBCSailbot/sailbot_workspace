import React from 'react';
import './VoyagePageStyles.css'; 

function VoyagePageTemplate({ desc }) {
    return (
        <div className="container">
            <div className="blankSpace"></div>
            <div className="descBox">
                <p className="desc">{desc}</p>
            </div>
        </div>
    );
}

export default VoyagePageTemplate;
