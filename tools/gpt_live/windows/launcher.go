// GPT-Live Windows launcher: unpack the bundled runtime and run the shared app.
package main

import (
	"archive/zip"
	"bytes"
	_ "embed"
	"fmt"
	"io"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"strings"
)

//go:embed payload.zip
var payload []byte

func unpack(data []byte, dir string) error {
	archive, err := zip.NewReader(bytes.NewReader(data), int64(len(data)))
	if err != nil {
		return err
	}
	for _, entry := range archive.File {
		name := strings.ReplaceAll(entry.Name, "\\", "/")
		if !filepath.IsLocal(name) || strings.Contains(name, ":") {
			return fmt.Errorf("invalid archive path")
		}
		target := filepath.Join(dir, filepath.FromSlash(name))
		if entry.FileInfo().IsDir() {
			if err := os.MkdirAll(target, 0700); err != nil {
				return err
			}
			continue
		}
		if err := os.MkdirAll(filepath.Dir(target), 0700); err != nil {
			return err
		}
		input, err := entry.Open()
		if err != nil {
			return err
		}
		output, err := os.OpenFile(target, os.O_CREATE|os.O_EXCL|os.O_WRONLY, 0600)
		if err != nil {
			input.Close()
			return err
		}
		_, copyErr := io.Copy(output, input)
		input.Close()
		closeErr := output.Close()
		if copyErr != nil {
			return copyErr
		}
		if closeErr != nil {
			return closeErr
		}
	}
	return nil
}

func run() error {
	dir, err := os.MkdirTemp("", "gpt-live-")
	if err != nil {
		return err
	}
	defer os.RemoveAll(dir)
	if err := unpack(payload, dir); err != nil {
		return err
	}
	command := exec.Command(filepath.Join(dir, "python.exe"), "-I", filepath.Join(dir, "server.py"), "--port", "0", "--open")
	command.Stdin, command.Stdout, command.Stderr = os.Stdin, os.Stdout, os.Stderr
	// Python receives the console's Ctrl+C too and handles graceful shutdown.
	interrupts := make(chan os.Signal, 1)
	signal.Notify(interrupts, os.Interrupt)
	defer signal.Stop(interrupts)
	fmt.Println("GPT-Live: opening your browser. Keep this window open; press Ctrl+C to exit.")
	return command.Run()
}

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, "Unable to start GPT-Live:", err)
		fmt.Println("Press Enter to close.")
		fmt.Scanln()
		os.Exit(1)
	}
}
