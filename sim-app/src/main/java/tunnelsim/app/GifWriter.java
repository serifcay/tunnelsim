package tunnelsim.app;

import javax.imageio.IIOImage;
import javax.imageio.ImageIO;
import javax.imageio.ImageTypeSpecifier;
import javax.imageio.ImageWriter;
import javax.imageio.metadata.IIOMetadata;
import javax.imageio.metadata.IIOMetadataNode;
import javax.imageio.stream.ImageOutputStream;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

/** Minimal looping animated GIF writer on top of the JDK's built-in GIF encoder. */
final class GifWriter implements AutoCloseable {
    private final ImageWriter writer;
    private final ImageOutputStream out;
    private final IIOMetadata metadata;

    GifWriter(File file, int delayMs) throws IOException {
        // Use the JDK's GIF encoder in sequence mode, one image per frame.
        writer = ImageIO.getImageWritersBySuffix("gif").next();
        out = ImageIO.createImageOutputStream(file);
        writer.setOutput(out);
        ImageTypeSpecifier type = ImageTypeSpecifier.createFromBufferedImageType(BufferedImage.TYPE_INT_RGB);
        metadata = writer.getDefaultImageMetadata(type, writer.getDefaultWriteParam());
        String format = metadata.getNativeMetadataFormatName();
        IIOMetadataNode root = (IIOMetadataNode) metadata.getAsTree(format);
        IIOMetadataNode gce = child(root, "GraphicControlExtension");
        gce.setAttribute("disposalMethod", "none");
        gce.setAttribute("userInputFlag", "FALSE");
        gce.setAttribute("transparentColorFlag", "FALSE");
        // Frame delay is stored in hundredths of a second.
        gce.setAttribute("delayTime", Integer.toString(Math.max(1, delayMs / 10)));
        gce.setAttribute("transparentColorIndex", "0");
        // The NETSCAPE2.0 extension makes the GIF loop forever.
        IIOMetadataNode app = new IIOMetadataNode("ApplicationExtension");
        app.setAttribute("applicationID", "NETSCAPE");
        app.setAttribute("authenticationCode", "2.0");
        app.setUserObject(new byte[]{1, 0, 0}); // loop forever
        child(root, "ApplicationExtensions").appendChild(app);
        metadata.setFromTree(format, root);
        writer.prepareWriteSequence(null);
    }

    /** Appends one frame. */
    void add(BufferedImage image) throws IOException {
        writer.writeToSequence(new IIOImage(image, null, metadata), writer.getDefaultWriteParam());
    }

    @Override
    public void close() throws IOException {
        writer.endWriteSequence();
        out.close();
    }

    /** Finds a metadata node by name, creating it if missing. */
    private static IIOMetadataNode child(IIOMetadataNode root, String name) {
        for (int i = 0; i < root.getLength(); i++) {
            if (root.item(i).getNodeName().equalsIgnoreCase(name)) {
                return (IIOMetadataNode) root.item(i);
            }
        }
        IIOMetadataNode node = new IIOMetadataNode(name);
        root.appendChild(node);
        return node;
    }
}
