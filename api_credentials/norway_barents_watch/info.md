# Info
This file contains a standard setup for the user specific yaml file containing login information to the
BarentsWatch API.

* To setup a client, first you have to create a free account at [https://www.barentswatch.no/]
* Next, you need to setup a client [here][https://www.barentswatch.no/minside/]
* Your client id will typically be on the format your.login@email.com:<CLIENT_ID>
* You will also need the client secret
* Create a new file in this folder called "credentials.yaml"

As a safety precaution, all yaml files are listed in the .gitignore file, so it does not become public by accident.

File: credentials.yaml
```yaml
credentials : {
  client_id : "<INSERT_YOUR_CLIENT_ID>",
  client_secret : "<INSERT_YOUR_CLIENT_SECRET>",
  scope : "ais",
  access_url : "https://id.barentswatch.no/connect/token",
  ais_url : "https://live.ais.barentswatch.no/v1/latest/combined"
}
```