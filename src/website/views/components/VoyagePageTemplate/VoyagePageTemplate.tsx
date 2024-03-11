import React from 'react';
import './VoyagePageStyles.css'; 

function VoyagePageTemplate({ image, header, desc }) {
    return (
        <div className="container">
            <div className="header">
                <img src={image} alt="Voyage" className="image" />
                <h1 className="headerText">{header}</h1>
            </div>
            <div className="descBox">
                <p className="desc">{desc}</p>
            </div>
        </div>
    );
}

export default VoyagePageTemplate;