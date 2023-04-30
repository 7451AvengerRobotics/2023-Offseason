package frc.lib.util;

public class GridNode {
    public Integer level;
    public Integer gamePiece;

    public GridNode(int level, int gamePiece){
        this.level = level;
        this.gamePiece = gamePiece;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result
                + ((level == null) ? 0 : level.hashCode());
        return result;
    }

    @Override
    public boolean equals(final Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
            
        final GridNode other = (GridNode) obj;
        if (level == null) {
            if (other.level != null)
                return false;
        } else if (!level.equals(other.level))
            return false;
        if (gamePiece == null) {
            if (other.gamePiece != null)
                return false;
        } else if (!gamePiece.equals(other.gamePiece))
                return false;            
        return true;
    }
}
