@aisships
Feature: Testing AISShips interface

  Scenario: Fetch AISShips data from the API
    Given I clear the database
    And I insert AISShips data into the database
    When I get all AISShip interface data
    Then the service success response is 200
    And the response data matches the aisship data in the database
