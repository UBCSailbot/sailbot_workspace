import React, { Component } from 'react';
import Checkbox from '@mui/material/Checkbox';
import FormGroup from '@mui/material/FormGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import FormControl from '@mui/material/FormControl';
import Paper from '@mui/material/Paper';

class Checkboxes extends React.Component {
  constructor(props) {
    super(props);

    this.handleButtonClick = this.handleButtonClick.bind(this);

    //initializing an array to track button states
    this.state = {
      //           button1 button2 button3
      buttonStates: [false, false, false],
    };
  }

  checkChecked(status, checkboxNumber) {
    if (!status) {
      console.log('hello, i am layer ' + checkboxNumber + '!');
    }
  }

  handleButtonClick(index) {
    let newArr = [...this.state.buttonStates];
    newArr[index] = !newArr[index];
    this.setState({ buttonStates: newArr });
    this.checkChecked(this.state.buttonStates[index], ++index);
  }

  render() {
    const { buttonStates } = this.state;

    return (
      <Paper elevation={0}>
        <FormControl component='fieldset'>
          <FormGroup aria-label='position' row>
            <FormControlLabel
              control={
                <Checkbox
                  checked={buttonStates[0]}
                  onChange={() => this.handleButtonClick(0)}
                />
              }
              label='Layer 1'
              labelPlacement='bottom'
            />
            <FormControlLabel
              control={
                <Checkbox
                  checked={buttonStates[1]}
                  onChange={() => this.handleButtonClick(1)}
                />
              }
              label='Layer 2'
              labelPlacement='bottom'
            />
            <FormControlLabel
              control={
                <Checkbox
                  checked={buttonStates[2]}
                  onChange={() => this.handleButtonClick(2)}
                />
              }
              label='Layer 3'
              labelPlacement='bottom'
            />
          </FormGroup>
        </FormControl>
      </Paper>
    );
  }
}

export default Checkboxes;
