## How I got this to work

* Install [rbenv](https://github.com/rbenv/rbenv#basic-github-checkout)

    $ git clone https://github.com/rbenv/rbenv.git ~/.rbenv
    $ cd ~/.rbenv && src/configure && make -C src

* Add `~/.rbenv/bin` to your `$PATH` for access to the `rbenv` command-line utility.

    $ echo 'export PATH="$HOME/.rbenv/bin:$PATH"' >> ~/.bash_profile

* Run `~/.rbenv/bin/rbenv init` and follow the instructions to set up rbenv integration with your shell.

* Install [ruby-build](https://github.com/rbenv/ruby-build#installation), which provides the rbenv install command that simplifies the process of installing new Ruby versions.

    $ mkdir -p "$(rbenv root)"/plugins
    $ git clone https://github.com/rbenv/ruby-build.git "$(rbenv root)"/plugins/ruby-build

* build ruby 2.4.4

    $ CONFIGURE_OPTS="--enable-shared" rbenv install 2.4.2

* activate ruby 2.4.2

    $ rbenv local 2.4.2

* install bundler

    $ gem install bundler

* run `bundle install` in this directory
* run `ruby ./Launcher` in this directory
