@echo off
setlocal

set IMAGE_NAME=racecar:latest

echo Building the %IMAGE_NAME% image...

docker buildx build --tag %IMAGE_NAME% . > ./build.log 2>&1

if %errorlevel% equ 0 (
    echo Docker build finished successfully.
    @REM del build.log
    docker image prune -f
) else (
    echo Docker build failed. Check `build.log` for details.
)

endlocal
