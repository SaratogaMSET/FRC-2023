/*
 * This Java source file was generated by the Gradle 'init' task.
 */
package testclient;

public class App {
    public static void main(String[] args) {
        FishClientNT c = new FishClientNT();
        c.start();
        // FIXME on a different thread, listen for filter flag changes, then re-init filter on change
    }
}
