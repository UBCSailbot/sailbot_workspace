@gps
Feature: Testing the GPS API

  Scenario: Fetch GPS data from the API
    Given I clear the database
    And I insert GPS data into the database
    When I get all GPS interface data
    Then the service success response is 200
    And the response data matches the data in the database
