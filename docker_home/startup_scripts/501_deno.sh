export DENO_INSTALL="$HOME/.deno"
export PATH="$PATH:$DENO_INSTALL/bin"
# if deno doesnt exist
if [ -z "$(command -v "deno")" ]
then
    # NOTE: DENO_VERSION is set in the Dockerfile
    # download it
    curl -fsSL https://deno.land/install.sh | sh -s v"$DENO_VERSION"
fi