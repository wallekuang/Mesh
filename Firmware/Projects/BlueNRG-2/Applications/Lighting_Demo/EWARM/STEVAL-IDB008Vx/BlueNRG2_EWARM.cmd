echo off

pushd "%1"

copy "%1\%2\Exe\%3" "%1\..\..\Binary\%3"

popd
