## How I got this to work

* Install [rbenv](https://github.com/rbenv/rbenv)
* Add rbenv to your path

    PATH=~/.rbenv/shims:$PATH

* Checkout our custom version of [COSMOS](https://github.com/contradict/COSMOS)

    $ cd ~/src
    $ git clone https://github.com/contradict/COSMOS.git

* Set the environment variable `COSMOS_DEVEL` to the full path to the checkout location

    $ export COSMOS_DEVEL=~/src/COSMOS

* run `bundle install` in this directory
* run `ruby ./Launcher` in this directory
