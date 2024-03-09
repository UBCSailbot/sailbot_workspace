import { When } from '@cucumber/cucumber';
import { api } from '../shared/classes/api';
import { GPS } from '../shared/endpoints';
import { AISShips } from '../shared/endpoints';
import { GlobalPath } from '../shared/endpoints';
import { LocalPath } from '../shared/endpoints';
import { Batteries } from '../shared/endpoints';
import { GenericSensors } from '../shared/endpoints';
import { WindSensors } from '../shared/endpoints';

When('I get all GPS interface data', async function () {
  this.lastResponse = await api.get(GPS, this.config);
});

When('I try to get all GPS interface data', async function () {
  this.lastResponse = await api.get(GPS, this.config, false);
});

When('I get all AISShip interface data', async function () {
  this.lastResponse = await api.get(AISShips, this.config);
});

When('I get all GlobalPath interface data', async function () {
  this.lastResponse = await api.get(GlobalPath, this.config);
});

When('I get all LocalPath interface data', async function () {
  this.lastResponse = await api.get(LocalPath, this.config);
});

When('I get all Batteries interface data', async function () {
  this.lastResponse = await api.get(Batteries, this.config);
});

When('I get all GenericSensors interface data', async function () {
  this.lastResponse = await api.get(GenericSensors, this.config);
});

When('I get all WindSensors interface data', async function () {
  this.lastResponse = await api.get(WindSensors, this.config);
});

export {};
